## Instructions for Running the Code

Follow these steps to get started:

1. **Clone or download the git repository**:
    ```bash
    git clone https://github.com/Travimadox/3096S-Pracs-WBBTRA001-MSHRUM006.git
    ```

2. The project folder you will need is located at: `3096S-Pracs-WBBTRA001-MSHRUM006/Prac2/Prac2_student`. Additionally, make sure you download and install **STMCubeIDE** which is the IDE used to program the practical.

3. **Setting up in STMCubeIDE**:
   - Launch `STMCubeIDE`.
   - Navigate to `File` --> `Import`.
   - Choose `Existing Code as Makefile Project` and click `Next`.
   - Click `Browse` and navigate to the project folder.
   - Select `MCU ARM GCC` as the Toolchain.
   - Click `Finish`.

4. **Test the Code**:
   - Build and then debug the code to test functionality.
   - The end result of all this should be code that achieves the following:

     | Description | Details |
     |-------------|---------|
     | On system start-up | SPI is used to write the aforementioned array of 8-bit integers to EEPROM (once off). |
     | On every timer interrupt | One of these 8-bit values must be read from EEPROM (using SPI) and then written to the eight red LEDs to produce a pattern. By default, the delay between interrupts should be 1 second. |
     | Pressing PA0 | Should switch the timer delay from 1 second to 0.5 seconds, or vice-versa. |

