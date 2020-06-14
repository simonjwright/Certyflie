target remote :4242
load
break __gnat_last_chance_handler
monitor semihosting enable
monitor semihosting ioclient 2

