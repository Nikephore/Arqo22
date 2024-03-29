#!/bin/bash

Ninicio=512+6
Npaso=64
Nfinal=$((Ninicio + 1024))

echo "Running normal_serie and normal_par..."

for((N=Ninicio; N<= Nfinal; N+=Npaso)); do
    echo "Damos una vuelta"
    echo "Tamanio de matriz: $N"
    normalserieTime=$(../normal_serie "$N" | grep 'time' | awk '{print $3}')
    normalparalelTime=$(../normal_par1 "$N" 4 | grep 'time' | awk '{print $3}')

    echo "$N $normalserieTime $normalparalelTime" >> ejercicio3.dat
done

