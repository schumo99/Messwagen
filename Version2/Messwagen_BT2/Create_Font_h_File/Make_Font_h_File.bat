@echo off
REM Create temp directory
mkdir \tmp 2> NUL

REM Create Px437_Wyse700a.bdf
REM Has to be called only once as a starting point to edit the font
otf2bdf\otf2bdf -r 72 -p 16 -o Px437_Wyse700a.bdf Px437_Wyse700a.ttf

REM Create a font with minimal needed characters
REM -.0123456789 Space
REM Special Characters
REM
REM The font was manualy generated with Fony.exe based on the bdf font created above
REM
bdfconv -y -1 -th 2 -tv 2 -v -f 2 -m "32-73" Px437_Wyse700a_Mini.bdf -o ..\u8x8_font_px437wyse700a_2x2_Mini.h  -n u8x8_font_px437wyse700a_2x2_Mini

Pause




