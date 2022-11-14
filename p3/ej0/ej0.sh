#!/bin/bash
#
#$ -S /bin/bash
#$ -cwd
#$ -o salida.out
#$ -j y
# Anadir hwloc al path
export PATH=$PATH:/share/apps/tools/hwloc/bin/
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/share/apps/tools/hwloc/lib64/
# Ejecutar lstopo con salida en formato png (genera un fichero con la imagen)
lstopo --output-format png > figure.png
