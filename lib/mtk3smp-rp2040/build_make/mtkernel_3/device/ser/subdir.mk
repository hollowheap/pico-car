################################################################################
# micro T-Kernel 3.0 BSP  makefile
################################################################################

OBJS += \
./mtkernel_3/device/ser/ser.o 

C_DEPS += \
./mtkernel_3/device/ser/ser.d 


# Each subdirectory must supply rules for building sources it contributes
mtkernel_3/device/ser/%.o: ../device/ser/%.c
	@echo 'Building file: $<'
	$(GCC) $(CFLAGS) -D$(TARGET) $(INCPATH) -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

ifeq ($(TARGET), _PICO_RP2040_)
-include mtkernel_3/device/ser/sysdepend/rp2040/subdir.mk
endif
