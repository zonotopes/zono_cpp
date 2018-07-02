@echo off
echo Testing zonotope on a default dataset having 10 generators in 3D
echo (The known volume for this dataset is 20.1948)
zonotope.exe

echo.
echo In order to test the zonotope library on a second dataset, 
echo please uncomment next line:
echo rem zonotope.exe zono_data\R5gen10.txt
