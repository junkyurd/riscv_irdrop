module heater_wrapper(
    input EN
);
    rare_heater heater_unit_inst(
        .EN(EN)
    );

endmodule