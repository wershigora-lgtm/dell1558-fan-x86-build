#define _CRT_SECURE_NO_WARNINGS
#include <windows.h>
#include <winioctl.h>
#include <stdio.h>
#include <conio.h>
#include <mmsystem.h>

#define IOCTL_DELL_READ_STATUS \
    CTL_CODE(FILE_DEVICE_UNKNOWN, 0x800, METHOD_BUFFERED, FILE_READ_ACCESS)

#define IOCTL_DELL_SET_STATE \
    CTL_CODE(FILE_DEVICE_UNKNOWN, 0x801, METHOD_BUFFERED, FILE_READ_ACCESS | FILE_WRITE_ACCESS)

#define IOCTL_DELL_READ_SENSOR \
    CTL_CODE(FILE_DEVICE_UNKNOWN, 0x802, METHOD_BUFFERED, FILE_READ_ACCESS)

#define IOCTL_DELL_READ_NOMINAL \
    CTL_CODE(FILE_DEVICE_UNKNOWN, 0x803, METHOD_BUFFERED, FILE_READ_ACCESS)

typedef struct _DELL_STATUS_RESULT
{
    DWORD State;
    DWORD Rpm;
    DWORD CpuNumber;
    DWORD StateValid;
    DWORD RpmValid;
} DELL_STATUS_RESULT;

typedef struct _DELL_SET_REQUEST
{
    DWORD State;
} DELL_SET_REQUEST;

typedef struct _DELL_SET_RESULT
{
    DWORD RequestedState;
    DWORD RawEax;
    DWORD CarryFlag;
    DWORD Valid;
} DELL_SET_RESULT;

typedef struct _DELL_SENSOR_REQUEST
{
    DWORD Sensor;
} DELL_SENSOR_REQUEST;

typedef struct _DELL_SENSOR_RESULT
{
    DWORD Sensor;
    DWORD Type;
    DWORD Temperature;
    DWORD TypeValid;
    DWORD TempValid;
} DELL_SENSOR_RESULT;

typedef struct _DELL_NOMINAL_REQUEST
{
    DWORD State;
} DELL_NOMINAL_REQUEST;

typedef struct _DELL_NOMINAL_RESULT
{
    DWORD State;
    DWORD NominalRpm;
    DWORD Valid;
} DELL_NOMINAL_RESULT;

typedef enum _CONTROL_MODE
{
    MODE_AUTO = 0,
    MODE_HOLD_LOW = 1,
    MODE_TARGET = 2
} CONTROL_MODE;

static BOOL
ReadStatus(
    HANDLE h,
    DELL_STATUS_RESULT *r
    )
{
    DWORD bytes = 0;

    ZeroMemory(r, sizeof(*r));

    return
        DeviceIoControl(
            h,
            IOCTL_DELL_READ_STATUS,
            NULL,
            0,
            r,
            sizeof(*r),
            &bytes,
            NULL
            ) &&
        bytes == sizeof(*r);
}

static BOOL
SetState(
    HANDLE h,
    DWORD state
    )
{
    DELL_SET_REQUEST request;
    DELL_SET_RESULT result;
    DWORD bytes = 0;

    request.State = state;
    ZeroMemory(&result, sizeof(result));

    if (!DeviceIoControl(
        h,
        IOCTL_DELL_SET_STATE,
        &request,
        sizeof(request),
        &result,
        sizeof(result),
        &bytes,
        NULL
        ))
    {
        return FALSE;
    }

    return
        bytes == sizeof(result) &&
        result.Valid == 1 &&
        result.CarryFlag == 0;
}

static BOOL
ReadSensor(
    HANDLE h,
    DWORD sensor,
    DELL_SENSOR_RESULT *r
    )
{
    DELL_SENSOR_REQUEST request;
    DWORD bytes = 0;

    request.Sensor = sensor;
    ZeroMemory(r, sizeof(*r));

    return
        DeviceIoControl(
            h,
            IOCTL_DELL_READ_SENSOR,
            &request,
            sizeof(request),
            r,
            sizeof(*r),
            &bytes,
            NULL
            ) &&
        bytes == sizeof(*r);
}

static BOOL
ReadNominal(
    HANDLE h,
    DWORD state,
    DWORD *rpm
    )
{
    DELL_NOMINAL_REQUEST request;
    DELL_NOMINAL_RESULT result;
    DWORD bytes = 0;

    request.State = state;
    ZeroMemory(&result, sizeof(result));

    if (!DeviceIoControl(
        h,
        IOCTL_DELL_READ_NOMINAL,
        &request,
        sizeof(request),
        &result,
        sizeof(result),
        &bytes,
        NULL
        ))
    {
        return FALSE;
    }

    if (
        bytes != sizeof(result) ||
        result.Valid != 1
       )
    {
        return FALSE;
    }

    *rpm = result.NominalRpm;
    return TRUE;
}

static BOOL
ReadMaxTemperature(
    HANDLE h,
    DWORD *maxTemp,
    DWORD *validCount
    )
{
    DWORD i;
    DELL_SENSOR_RESULT r;

    *maxTemp = 0;
    *validCount = 0;

    for (i = 0; i < 10; ++i)
    {
        if (!ReadSensor(h, i, &r))
        {
            continue;
        }

        if (r.TempValid != 1)
        {
            continue;
        }

        ++(*validCount);

        if (r.Temperature > *maxTemp)
        {
            *maxTemp = r.Temperature;
        }
    }

    return *validCount > 0;
}

static const char *
ModeName(
    CONTROL_MODE mode
    )
{
    if (mode == MODE_HOLD_LOW)
    {
        return "HOLD-LOW";
    }

    if (mode == MODE_TARGET)
    {
        return "TARGET";
    }

    return "AUTO";
}

static VOID
PrintHelp(VOID)
{
    printf("\nControls:\n");
    printf("  A       AUTO: stop overriding BIOS\n");
    printf("  L       HOLD-LOW: poll BIOS and request LOW only when needed\n");
    printf("  T       TARGET-RPM mode\n");
    printf("  + / -   target RPM +/- 100\n");
    printf("  ] / [   poll interval +/- 5 ms\n");
    printf("  2       request HIGH once, then AUTO\n");
    printf("  H       show help\n");
    printf("  Q       request HIGH once and quit\n\n");
}

int __cdecl
main(void)
{
    HANDLE h;
    CONTROL_MODE mode = MODE_AUTO;
    DELL_STATUS_RESULT status;
    DWORD lowNominal = 0;
    DWORD highNominal = 0;
    DWORD targetRpm = 3500;
    DWORD pulseMs = 10;
    DWORD hysteresis = 120;
    DWORD tempLimit = 85;
    DWORD maxTemp = 0;
    DWORD validTempCount = 0;
    DWORD missingTempScans = 0;
    ULONGLONG lastControl = 0;
    ULONGLONG lastStatus = 0;
    ULONGLONG lastTemp = 0;
    BOOL running = TRUE;

    printf("Dell Studio 1558 Fan Control - x86\n");
    printf("Fan OFF is blocked in the driver.\n");
    printf("BIOS automatic-control disable commands are NOT used.\n");

    h =
        CreateFileA(
            "\\\\.\\DellFanControl",
            GENERIC_READ | GENERIC_WRITE,
            FILE_SHARE_READ | FILE_SHARE_WRITE,
            NULL,
            OPEN_EXISTING,
            0,
            NULL
            );

    if (h == INVALID_HANDLE_VALUE)
    {
        printf(
            "ERROR: cannot open DellFanControl device (Win32 %lu).\n",
            GetLastError()
            );
        printf(
            "Start DellFanControl.sys first and run this app as Administrator.\n"
            );
        return 2;
    }

    if (ReadNominal(h, 1, &lowNominal))
    {
        printf("Nominal LOW : %lu RPM\n", lowNominal);
    }
    else
    {
        printf("Nominal LOW : unavailable\n");
    }

    if (ReadNominal(h, 2, &highNominal))
    {
        printf("Nominal HIGH: %lu RPM\n", highNominal);
    }
    else
    {
        printf("Nominal HIGH: unavailable\n");
    }

    if (
        lowNominal > 0 &&
        highNominal > lowNominal
       )
    {
        targetRpm =
            lowNominal +
            ((highNominal - lowNominal) / 2);
    }

    if (ReadMaxTemperature(
        h,
        &maxTemp,
        &validTempCount
        ))
    {
        printf(
            "Temperature telemetry: %lu sensor(s), current max %lu C\n",
            validTempCount,
            maxTemp
            );
    }
    else
    {
        printf(
            "Temperature telemetry: unavailable. BIOS remains active as fallback.\n"
            );
    }

    timeBeginPeriod(1);

    PrintHelp();

    while (running)
    {
        ULONGLONG now =
            GetTickCount64();

        if (now - lastTemp >= 1000)
        {
            lastTemp = now;

            if (ReadMaxTemperature(
                h,
                &maxTemp,
                &validTempCount
                ))
            {
                missingTempScans = 0;

                if (
                    maxTemp >= tempLimit &&
                    mode != MODE_AUTO
                   )
                {
                    SetState(h, 2);
                    mode = MODE_AUTO;

                    printf(
                        "\nSAFETY: %lu C >= %lu C. HIGH requested; AUTO restored.\n",
                        maxTemp,
                        tempLimit
                        );
                }
            }
            else
            {
                ++missingTempScans;

                if (
                    missingTempScans >= 3 &&
                    mode != MODE_AUTO
                   )
                {
                    /*
                     * Telemetry loss does not disable BIOS; nevertheless,
                     * stop our repeated LOW requests immediately.
                     */
                    SetState(h, 2);
                    mode = MODE_AUTO;

                    printf(
                        "\nSAFETY: temperature telemetry lost. HIGH requested; AUTO restored.\n"
                        );
                }
            }
        }

        if (mode == MODE_HOLD_LOW)
        {
            if (
                validTempCount > 0 &&
                now - lastControl >= pulseMs
               )
            {
                /*
                 * Quiet hold:
                 * poll the Dell-reported state frequently, but do NOT
                 * keep hammering LOW while the fan is already in state 1.
                 * Only counter the BIOS after it changes away from LOW.
                 */
                if (
                    ReadStatus(h, &status) &&
                    status.StateValid == 1 &&
                    status.State != 1
                   )
                {
                    SetState(h, 1);
                }

                lastControl = now;
            }
        }
        else if (mode == MODE_TARGET)
        {
            if (
                validTempCount > 0 &&
                now - lastControl >= pulseMs
               )
            {
                if (
                    ReadStatus(h, &status) &&
                    status.RpmValid == 1
                   )
                {
                    if (
                        status.Rpm >
                        targetRpm + hysteresis
                       )
                    {
                        SetState(h, 1);
                    }
                    else if (
                        status.Rpm + hysteresis <
                        targetRpm
                        )
                    {
                        SetState(h, 2);
                    }
                }

                lastControl = now;
            }
        }

        if (now - lastStatus >= 1000)
        {
            lastStatus = now;

            if (ReadStatus(h, &status))
            {
                printf(
                    "%-8s  ",
                    ModeName(mode)
                    );

                if (status.StateValid)
                {
                    printf("state=%lu  ", status.State);
                }
                else
                {
                    printf("state=?  ");
                }

                if (status.RpmValid)
                {
                    printf("rpm=%lu  ", status.Rpm);
                }
                else
                {
                    printf("rpm=?  ");
                }

                printf(
                    "target=%lu  poll=%lums  ",
                    targetRpm,
                    pulseMs
                    );

                if (validTempCount > 0)
                {
                    printf("Tmax=%luC", maxTemp);
                }
                else
                {
                    printf("Tmax=?");
                }

                printf("\n");
            }
        }

        if (_kbhit())
        {
            int ch =
                _getch();

            if (
                ch >= 'a' &&
                ch <= 'z'
               )
            {
                ch -= ('a' - 'A');
            }

            if (ch == 'A')
            {
                mode = MODE_AUTO;
                printf(
                    "\nAUTO: override pulses stopped.\n"
                    );
            }
            else if (ch == 'L')
            {
                if (validTempCount == 0)
                {
                    printf(
                        "\nLOCKED: no valid temperature telemetry.\n"
                        );
                }
                else
                {
                    mode = MODE_HOLD_LOW;
                    lastControl = 0;

                    printf(
                        "\nHOLD-LOW: polling every %lu ms; LOW only when BIOS leaves state 1.\n",
                        pulseMs
                        );
                }
            }
            else if (ch == 'T')
            {
                if (validTempCount == 0)
                {
                    printf(
                        "\nLOCKED: no valid temperature telemetry.\n"
                        );
                }
                else
                {
                    mode = MODE_TARGET;
                    lastControl = 0;

                    printf(
                        "\nTARGET mode: %lu RPM.\n",
                        targetRpm
                        );
                }
            }
            else if (ch == '+')
            {
                targetRpm += 100;

                if (
                    highNominal > 0 &&
                    targetRpm > highNominal
                   )
                {
                    targetRpm = highNominal;
                }

                printf(
                    "\nTarget = %lu RPM\n",
                    targetRpm
                    );
            }
            else if (ch == '-')
            {
                if (targetRpm > 100)
                {
                    targetRpm -= 100;
                }

                if (
                    lowNominal > 0 &&
                    targetRpm < lowNominal
                   )
                {
                    targetRpm = lowNominal;
                }

                printf(
                    "\nTarget = %lu RPM\n",
                    targetRpm
                    );
            }
            else if (ch == ']')
            {
                if (pulseMs < 1000)
                {
                    pulseMs += 5;
                }

                printf(
                    "\nPoll interval = %lu ms\n",
                    pulseMs
                    );
            }
            else if (ch == '[')
            {
                if (pulseMs > 5)
                {
                    pulseMs -= 5;
                }

                printf(
                    "\nPoll interval = %lu ms\n",
                    pulseMs
                    );
            }
            else if (ch == '2')
            {
                SetState(h, 2);
                mode = MODE_AUTO;

                printf(
                    "\nHIGH requested once; AUTO mode.\n"
                    );
            }
            else if (ch == 'H')
            {
                PrintHelp();
            }
            else if (ch == 'Q')
            {
                SetState(h, 2);
                mode = MODE_AUTO;
                running = FALSE;
            }
        }

        Sleep(1);
    }

    timeEndPeriod(1);
    CloseHandle(h);

    printf(
        "\nController closed. BIOS remains active.\n"
        );

    return 0;
}
