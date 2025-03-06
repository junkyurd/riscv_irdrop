module temp_sensor_wrapper(
    inout Sense_Hi,
    inout Sense_Lo,
    inout Source_Hi,
    inout Source_Lo,
    input SEL
);
    rare_sensor temp_sensor_inst(
        .Sense_Hi(Sense_Hi),
        .Sense_Lo(Sense_Lo),
        .Source_Hi(Source_Hi),
        .Source_Lo(Source_Lo),
        .SEL(SEL)
    );
endmodule