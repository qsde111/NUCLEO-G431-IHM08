# NUCLEO-G431-IHM08 Architecture Bootstrap

This project now uses three non-Core layers:

- `BSP/`: HAL/LL-facing board and peripheral glue.
- `Components/`: protocol/math logic without HAL coupling.
- `App/`: application scheduler and module composition.

Current communication path:

1. `MotorApp_Loop()` in `App/motor_app.c`
2. `Mt6835_ReadRaw21()` in `Components/mt6835.c`
3. `BspSpi3Fast_Transfer8()` in `BSP/bsp_spi3_fast.c`
4. `JustFloat_Pack4()` in `Components/justfloat.c`
5. `BspUartDma_Send()` in `BSP/bsp_uart_dma.c`

## Build integration

- `.cproject` includes `App`, `BSP`, `Components` as source paths and include paths.
- `Debug/sources.mk`, `Debug/makefile`, and `Debug/objects.list` are also patched for immediate makefile compatibility.

If CubeIDE regenerates build files, keep `.cproject` changes and run a full clean/build so generated `Debug/*` files re-align with the new source paths.
