#rename 
set _my_datetime=%date%_%time%
set _my_datetime=%_my_datetime: =_%
set _my_datetime=%_my_datetime::=%
set _my_datetime=%_my_datetime:/=_%
set _my_datetime=%_my_datetime:.=_%


REN "C:\Users\John\Desktop\Dexter Industries\Programming\EV3_Dexter_Industries_Sensor_Development\EV3_Dexter_Industries_Sensors\Dexter.ev3b" Dexter%_my_datetime%.zip
REN Dexter.zip Dexter.ev3b