#include <ntddk.h>

#define DEVICE_NAME L"\\Device\\DellFanControl"
#define DOS_NAME    L"\\DosDevices\\DellFanControl"

#define IOCTL_DELL_READ_STATUS \
    CTL_CODE(FILE_DEVICE_UNKNOWN, 0x800, METHOD_BUFFERED, FILE_READ_ACCESS)

#define IOCTL_DELL_SET_STATE \
    CTL_CODE(FILE_DEVICE_UNKNOWN, 0x801, METHOD_BUFFERED, FILE_READ_ACCESS | FILE_WRITE_ACCESS)

#define IOCTL_DELL_READ_SENSOR \
    CTL_CODE(FILE_DEVICE_UNKNOWN, 0x802, METHOD_BUFFERED, FILE_READ_ACCESS)

#define IOCTL_DELL_READ_NOMINAL \
    CTL_CODE(FILE_DEVICE_UNKNOWN, 0x803, METHOD_BUFFERED, FILE_READ_ACCESS)

typedef struct _SMM_RESULT
{
    ULONG Eax;
    ULONG Ebx;
    ULONG Ecx;
    ULONG Edx;
    ULONG Esi;
    ULONG Edi;
    ULONG Carry;
} SMM_RESULT, *PSMM_RESULT;

typedef struct _DELL_STATUS_RESULT
{
    ULONG State;
    ULONG Rpm;
    ULONG CpuNumber;
    ULONG StateValid;
    ULONG RpmValid;
} DELL_STATUS_RESULT, *PDELL_STATUS_RESULT;

typedef struct _DELL_SET_REQUEST
{
    ULONG State;
} DELL_SET_REQUEST, *PDELL_SET_REQUEST;

typedef struct _DELL_SET_RESULT
{
    ULONG RequestedState;
    ULONG RawEax;
    ULONG CarryFlag;
    ULONG Valid;
} DELL_SET_RESULT, *PDELL_SET_RESULT;

typedef struct _DELL_SENSOR_REQUEST
{
    ULONG Sensor;
} DELL_SENSOR_REQUEST, *PDELL_SENSOR_REQUEST;

typedef struct _DELL_SENSOR_RESULT
{
    ULONG Sensor;
    ULONG Type;
    ULONG Temperature;
    ULONG TypeValid;
    ULONG TempValid;
} DELL_SENSOR_RESULT, *PDELL_SENSOR_RESULT;

typedef struct _DELL_NOMINAL_REQUEST
{
    ULONG State;
} DELL_NOMINAL_REQUEST, *PDELL_NOMINAL_REQUEST;

typedef struct _DELL_NOMINAL_RESULT
{
    ULONG State;
    ULONG NominalRpm;
    ULONG Valid;
} DELL_NOMINAL_RESULT, *PDELL_NOMINAL_RESULT;

DRIVER_INITIALIZE DriverEntry;
DRIVER_UNLOAD DellUnload;
DRIVER_DISPATCH DellCreateClose;
DRIVER_DISPATCH DellUnsupported;
DRIVER_DISPATCH DellDeviceControl;

static FAST_MUTEX gSmmMutex;

static NTSTATUS
CompleteIrp(
    PIRP Irp,
    NTSTATUS Status,
    ULONG_PTR Information
    )
{
    Irp->IoStatus.Status = Status;
    Irp->IoStatus.Information = Information;
    IoCompleteRequest(Irp, IO_NO_INCREMENT);
    return Status;
}

static VOID
DellSmmCallPinned(
    ULONG Command,
    ULONG EbxInput,
    PSMM_RESULT Result
    )
{
    ULONG eaxValue = Command;
    ULONG ebxValue = EbxInput;
    ULONG ecxValue = 0;
    ULONG edxValue = 0;
    ULONG esiValue = 0;
    ULONG ediValue = 0;
    UCHAR carry = 0xFF;

    __asm
    {
        push ebx
        push esi
        push edi

        mov eax, eaxValue
        mov ebx, ebxValue
        mov ecx, ecxValue
        mov edx, edxValue
        mov esi, esiValue
        mov edi, ediValue

        out 0B2h, al
        out 084h, al

        setc carry

        mov eaxValue, eax
        mov ebxValue, ebx
        mov ecxValue, ecx
        mov edxValue, edx
        mov esiValue, esi
        mov ediValue, edi

        pop edi
        pop esi
        pop ebx
    }

    Result->Eax = eaxValue;
    Result->Ebx = ebxValue;
    Result->Ecx = ecxValue;
    Result->Edx = edxValue;
    Result->Esi = esiValue;
    Result->Edi = ediValue;
    Result->Carry = (ULONG)carry;
}

static BOOLEAN
DellReadIsValid(
    ULONG OriginalCommand,
    PSMM_RESULT Result
    )
{
    if (Result->Carry != 0)
    {
        return FALSE;
    }

    if (Result->Eax == OriginalCommand)
    {
        return FALSE;
    }

    if ((Result->Eax & 0xFFFF) == 0xFFFF)
    {
        return FALSE;
    }

    return TRUE;
}

static BOOLEAN
PinCpu0(
    PKAFFINITY OldAffinity,
    PULONG CpuNumber
    )
{
    *OldAffinity =
        KeSetSystemAffinityThreadEx((KAFFINITY)1);

    *CpuNumber =
        KeGetCurrentProcessorNumber();

    if (*CpuNumber != 0)
    {
        KeRevertToUserAffinityThreadEx(*OldAffinity);
        return FALSE;
    }

    return TRUE;
}

static VOID
DellReadStatus(
    PDELL_STATUS_RESULT Result
    )
{
    KAFFINITY oldAffinity;
    ULONG cpuNumber;
    SMM_RESULT stateResult;
    SMM_RESULT rpmResult;

    RtlZeroMemory(Result, sizeof(*Result));
    Result->State = 0xFFFFFFFF;
    Result->CpuNumber = 0xFFFFFFFF;

    ExAcquireFastMutex(&gSmmMutex);

    if (!PinCpu0(&oldAffinity, &cpuNumber))
    {
        ExReleaseFastMutex(&gSmmMutex);
        return;
    }

    Result->CpuNumber = cpuNumber;

    DellSmmCallPinned(
        0x000000A3,
        0,
        &stateResult
        );

    DellSmmCallPinned(
        0x000002A3,
        0,
        &rpmResult
        );

    KeRevertToUserAffinityThreadEx(oldAffinity);
    ExReleaseFastMutex(&gSmmMutex);

    if (DellReadIsValid(0x000000A3, &stateResult))
    {
        Result->State = stateResult.Eax & 0xFF;
        Result->StateValid = 1;
    }

    if (DellReadIsValid(0x000002A3, &rpmResult))
    {
        Result->Rpm = rpmResult.Eax & 0xFFFF;
        Result->RpmValid = 1;
    }
}

static VOID
DellSetState(
    ULONG RequestedState,
    PDELL_SET_RESULT Result
    )
{
    KAFFINITY oldAffinity;
    ULONG cpuNumber;
    SMM_RESULT setResult;

    RtlZeroMemory(Result, sizeof(*Result));
    Result->RequestedState = RequestedState;
    Result->CarryFlag = 0xFFFFFFFF;

    /*
     * Final safety rule:
     *   state 0 (fan OFF) is never exposed.
     *   only LOW (1) and HIGH (2) can be requested.
     * BIOS automatic-control disable commands are not implemented.
     */
    if (RequestedState != 1 && RequestedState != 2)
    {
        return;
    }

    ExAcquireFastMutex(&gSmmMutex);

    if (!PinCpu0(&oldAffinity, &cpuNumber))
    {
        ExReleaseFastMutex(&gSmmMutex);
        return;
    }

    DellSmmCallPinned(
        0x000001A3,
        (RequestedState & 0xFF) << 8,
        &setResult
        );

    KeRevertToUserAffinityThreadEx(oldAffinity);
    ExReleaseFastMutex(&gSmmMutex);

    Result->RawEax = setResult.Eax;
    Result->CarryFlag = setResult.Carry;

    if (setResult.Carry == 0)
    {
        Result->Valid = 1;
    }
}

static VOID
DellReadSensor(
    ULONG Sensor,
    PDELL_SENSOR_RESULT Result
    )
{
    KAFFINITY oldAffinity;
    ULONG cpuNumber;
    SMM_RESULT typeResult;
    SMM_RESULT tempResult;
    ULONG temp;

    RtlZeroMemory(Result, sizeof(*Result));
    Result->Sensor = Sensor;
    Result->Type = 0xFFFFFFFF;
    Result->Temperature = 0xFFFFFFFF;

    if (Sensor > 9)
    {
        return;
    }

    ExAcquireFastMutex(&gSmmMutex);

    if (!PinCpu0(&oldAffinity, &cpuNumber))
    {
        ExReleaseFastMutex(&gSmmMutex);
        return;
    }

    DellSmmCallPinned(
        0x000011A3,
        Sensor & 0xFF,
        &typeResult
        );

    DellSmmCallPinned(
        0x000010A3,
        Sensor & 0xFF,
        &tempResult
        );

    KeRevertToUserAffinityThreadEx(oldAffinity);
    ExReleaseFastMutex(&gSmmMutex);

    temp = tempResult.Eax & 0xFF;

    if (DellReadIsValid(0x000011A3, &typeResult))
    {
        Result->Type = typeResult.Eax & 0xFF;
        Result->TypeValid = 1;
    }

    if (
        DellReadIsValid(0x000010A3, &tempResult) &&
        temp <= 127
       )
    {
        Result->Temperature = temp;
        Result->TempValid = 1;
    }
}

static VOID
DellReadNominal(
    ULONG State,
    PDELL_NOMINAL_RESULT Result
    )
{
    KAFFINITY oldAffinity;
    ULONG cpuNumber;
    SMM_RESULT nominalResult;

    RtlZeroMemory(Result, sizeof(*Result));
    Result->State = State;

    if (State > 3)
    {
        return;
    }

    ExAcquireFastMutex(&gSmmMutex);

    if (!PinCpu0(&oldAffinity, &cpuNumber))
    {
        ExReleaseFastMutex(&gSmmMutex);
        return;
    }

    DellSmmCallPinned(
        0x000004A3,
        (State & 0xFF) << 8,
        &nominalResult
        );

    KeRevertToUserAffinityThreadEx(oldAffinity);
    ExReleaseFastMutex(&gSmmMutex);

    if (DellReadIsValid(0x000004A3, &nominalResult))
    {
        Result->NominalRpm = nominalResult.Eax & 0xFFFF;
        Result->Valid = 1;
    }
}

NTSTATUS
DellCreateClose(
    PDEVICE_OBJECT DeviceObject,
    PIRP Irp
    )
{
    UNREFERENCED_PARAMETER(DeviceObject);
    return CompleteIrp(Irp, STATUS_SUCCESS, 0);
}

NTSTATUS
DellUnsupported(
    PDEVICE_OBJECT DeviceObject,
    PIRP Irp
    )
{
    UNREFERENCED_PARAMETER(DeviceObject);
    return CompleteIrp(
        Irp,
        STATUS_INVALID_DEVICE_REQUEST,
        0
        );
}

NTSTATUS
DellDeviceControl(
    PDEVICE_OBJECT DeviceObject,
    PIRP Irp
    )
{
    PIO_STACK_LOCATION stack;
    ULONG code;

    UNREFERENCED_PARAMETER(DeviceObject);

    stack = IoGetCurrentIrpStackLocation(Irp);
    code = stack->Parameters.DeviceIoControl.IoControlCode;

    if (code == IOCTL_DELL_READ_STATUS)
    {
        PDELL_STATUS_RESULT result;

        if (
            stack->Parameters.DeviceIoControl.OutputBufferLength
            < sizeof(DELL_STATUS_RESULT)
           )
        {
            return CompleteIrp(
                Irp,
                STATUS_BUFFER_TOO_SMALL,
                0
                );
        }

        result =
            (PDELL_STATUS_RESULT)
            Irp->AssociatedIrp.SystemBuffer;

        DellReadStatus(result);

        return CompleteIrp(
            Irp,
            STATUS_SUCCESS,
            sizeof(DELL_STATUS_RESULT)
            );
    }

    if (code == IOCTL_DELL_SET_STATE)
    {
        PDELL_SET_REQUEST request;
        PDELL_SET_RESULT result;
        ULONG requestedState;

        if (
            stack->Parameters.DeviceIoControl.InputBufferLength
            < sizeof(DELL_SET_REQUEST) ||
            stack->Parameters.DeviceIoControl.OutputBufferLength
            < sizeof(DELL_SET_RESULT)
           )
        {
            return CompleteIrp(
                Irp,
                STATUS_BUFFER_TOO_SMALL,
                0
                );
        }

        request =
            (PDELL_SET_REQUEST)
            Irp->AssociatedIrp.SystemBuffer;

        requestedState = request->State;

        result =
            (PDELL_SET_RESULT)
            Irp->AssociatedIrp.SystemBuffer;

        DellSetState(requestedState, result);

        return CompleteIrp(
            Irp,
            STATUS_SUCCESS,
            sizeof(DELL_SET_RESULT)
            );
    }

    if (code == IOCTL_DELL_READ_SENSOR)
    {
        PDELL_SENSOR_REQUEST request;
        PDELL_SENSOR_RESULT result;
        ULONG sensor;

        if (
            stack->Parameters.DeviceIoControl.InputBufferLength
            < sizeof(DELL_SENSOR_REQUEST) ||
            stack->Parameters.DeviceIoControl.OutputBufferLength
            < sizeof(DELL_SENSOR_RESULT)
           )
        {
            return CompleteIrp(
                Irp,
                STATUS_BUFFER_TOO_SMALL,
                0
                );
        }

        request =
            (PDELL_SENSOR_REQUEST)
            Irp->AssociatedIrp.SystemBuffer;

        sensor = request->Sensor;

        result =
            (PDELL_SENSOR_RESULT)
            Irp->AssociatedIrp.SystemBuffer;

        DellReadSensor(sensor, result);

        return CompleteIrp(
            Irp,
            STATUS_SUCCESS,
            sizeof(DELL_SENSOR_RESULT)
            );
    }

    if (code == IOCTL_DELL_READ_NOMINAL)
    {
        PDELL_NOMINAL_REQUEST request;
        PDELL_NOMINAL_RESULT result;
        ULONG state;

        if (
            stack->Parameters.DeviceIoControl.InputBufferLength
            < sizeof(DELL_NOMINAL_REQUEST) ||
            stack->Parameters.DeviceIoControl.OutputBufferLength
            < sizeof(DELL_NOMINAL_RESULT)
           )
        {
            return CompleteIrp(
                Irp,
                STATUS_BUFFER_TOO_SMALL,
                0
                );
        }

        request =
            (PDELL_NOMINAL_REQUEST)
            Irp->AssociatedIrp.SystemBuffer;

        state = request->State;

        result =
            (PDELL_NOMINAL_RESULT)
            Irp->AssociatedIrp.SystemBuffer;

        DellReadNominal(state, result);

        return CompleteIrp(
            Irp,
            STATUS_SUCCESS,
            sizeof(DELL_NOMINAL_RESULT)
            );
    }

    return CompleteIrp(
        Irp,
        STATUS_INVALID_DEVICE_REQUEST,
        0
        );
}

VOID
DellUnload(
    PDRIVER_OBJECT DriverObject
    )
{
    UNICODE_STRING dosName;

    RtlInitUnicodeString(&dosName, DOS_NAME);
    IoDeleteSymbolicLink(&dosName);

    if (DriverObject->DeviceObject != NULL)
    {
        IoDeleteDevice(DriverObject->DeviceObject);
    }
}

NTSTATUS
DriverEntry(
    PDRIVER_OBJECT DriverObject,
    PUNICODE_STRING RegistryPath
    )
{
    UNICODE_STRING deviceName;
    UNICODE_STRING dosName;
    PDEVICE_OBJECT deviceObject = NULL;
    NTSTATUS status;
    ULONG i;

    UNREFERENCED_PARAMETER(RegistryPath);

    ExInitializeFastMutex(&gSmmMutex);

    for (i = 0; i <= IRP_MJ_MAXIMUM_FUNCTION; ++i)
    {
        DriverObject->MajorFunction[i] = DellUnsupported;
    }

    DriverObject->MajorFunction[IRP_MJ_CREATE] = DellCreateClose;
    DriverObject->MajorFunction[IRP_MJ_CLOSE] = DellCreateClose;
    DriverObject->MajorFunction[IRP_MJ_DEVICE_CONTROL] = DellDeviceControl;
    DriverObject->DriverUnload = DellUnload;

    RtlInitUnicodeString(&deviceName, DEVICE_NAME);

    status =
        IoCreateDevice(
            DriverObject,
            0,
            &deviceName,
            FILE_DEVICE_UNKNOWN,
            FILE_DEVICE_SECURE_OPEN,
            FALSE,
            &deviceObject
            );

    if (!NT_SUCCESS(status))
    {
        return status;
    }

    deviceObject->Flags |= DO_BUFFERED_IO;

    RtlInitUnicodeString(&dosName, DOS_NAME);

    status =
        IoCreateSymbolicLink(
            &dosName,
            &deviceName
            );

    if (!NT_SUCCESS(status))
    {
        IoDeleteDevice(deviceObject);
        return status;
    }

    deviceObject->Flags &= ~DO_DEVICE_INITIALIZING;
    return STATUS_SUCCESS;
}
