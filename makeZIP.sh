#!/bin/bash

# Script to make the process of zipping the work automatic.
# Arguments: number of the assignment with two digits

FOLDER=../Garcia_P$1

mkdir $FOLDER

echo "Selecting the right files for you..."

cp -r inc $FOLDER/inc
cp -r src $FOLDER/src
cp -r Informes/I_$1.pdf $FOLDER/Informe_$1.pdf
cp -r Cuestionarios/C_$1.pdf $FOLDER/Cuestionario_$1.pdf

echo "Zipping everything..."

zip -r $FOLDER.zip $FOLDER

rm -r $FOLDER

echo "Done! Do not forget to send the zip to the teacher :)"
