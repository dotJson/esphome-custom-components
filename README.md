<!-- <a href="https://www.buymeacoffee.com/slimcdk"><img src="https://img.buymeacoffee.com/button-api/?text=Buy me a pizza&emoji=🍕&slug=slimcdk&button_colour=FFDD00&font_colour=000000&font_family=Cookie&outline_colour=000000&coffee_colour=ffffff" /></a> -->

# Custom components for ESPHome
```yaml
external_components:
  - source: github://dotJson/esphome-custom-components
    components: [ <component1>, <component2>, ... ]
```

## Components

- [tmc2209](esphome/components/tmc2209/README.md) :: ADI Trinamic stepper driver.

## Acknowledgements
This version graciously forked from the amazing work of [slimcdk](https://github.com/slimcdk/esphome-custom-components). Consider this derivative work addressing some issues I encountered when utilizing UART mode, some additional retry attempts on UART error and general compatibility concerns shown when compiling in ESPHome 2026.8.2. This project includes code from the Analog Devices Inc. [TMC-API codebase](https://github.com/analogdevicesinc/TMC-API), licensed under the MIT License.
