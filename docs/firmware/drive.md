# Drive
Drive.c includes motor drive related functions, 

## Variables
```{eval-rst}
.. doxygenvariable:: enc_angle_int
.. doxygenvariable:: electrical_angle_offset
```


## Functions

### Control
```{eval-rst}
.. doxygenfunction:: enable_foc_loop
.. doxygenfunction:: disable_foc_loop
```

### Calibration and Diagnostic

```{eval-rst}
.. doxygenfunction:: estimate_phase_resistance
.. doxygenfunction:: check_supply_voltage
.. doxygenfunction:: calibrate_encoder
```

### Gate drive
```{eval-rst}
.. doxygenfunction:: set_duty_phase_A
.. doxygenfunction:: set_duty_phase_B
.. doxygenfunction:: set_duty_phase_C
.. doxygenfunction:: set_duty_phases
```