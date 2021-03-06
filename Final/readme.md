# STM32H747I-DISCO

## Dual-Core Changes

This TouchGFX Board Setup (TBS) supports the dual-core functionality of the STM32H747I-DISCO.
For this reason, it is important to understand that there are two separate sub-projects for each compiler:
- A CM4 project
- A CM7 project

Since TouchGFX is running on the CM7, its code is located within the CM7 folder inside the project structure:

- π CM4
    - πCore
- π CM7
    - π Core
    - π TouchGFX


The CM4 and CM7 sub-project are split up as below:
- π STM32CubeIDE
    - π CM4
    - π CM7
