#!/bin/bash

# Change the working directory to the build directory from the directory of the script

cd "$(dirname "$0")/.." || exit 1

mkdir -p pdbs
mkdir -p pdbs/stp4
mkdir -p pdbs/stp5
mkdir -p pdbs/toh
mkdir -p data
mkdir -p data/stp4
mkdir -p data/stp5
mkdir -p data/toh
mkdir -p temp

cd "src/bin/release" || exit 1

echo "Running STP4 with MD and 4+4+4+4 for all algorithms"
for value in BAE A IDA AIDA RAIDA PEMBAE PEMM PEMA RPEMA; do
    ./embhs -d STP4 -h MD -i 0 -n 100 -a "$value" -p ../../../pdbs/stp4 -t ../../../temp -e 96 -r 96 -v info > ../../../data/stp4/STP4_"$value"_MD.out
    ./embhs -d STP4 -h 4+4+4+4 -i 0 -n 100 -a "$value" -p ../../../pdbs/stp4 -t ../../../temp -e 96 -r 96 -v info > ../../../data/stp4/STP4_"$value"_PDB.out
done

echo "Running STP4 with 6+6+6+6 for all algorithms"
for value in AIDA RAIDA PEMBAE; do
    ./embhs -d STP5 -h 6+6+6+6 -i 0 -n 50 -a "$value" -p ../../../pdbs/stp5 -t ../../../temp -e 96 -r 96 -v info > ../../../data/stp5/STP5_"$value"_96.out
done

echo "Running TOH with 16+4 for all algorithms"
for value in PEMBAE PEMM PEMA RPEMA; do
    ./embhs -d TOH -h 4 -i 0 -n 20 -a "$value" -p ../../../pdbs/toh -t ../../../temp -e 96 -r 96 -v info > ../../../data/toh/TOH_"$value".out
done

echo "Running thread count ablation study"
for outer in 4 36 45 48; do
    for value in 1 16 32 48 64 80; do
        ./embhs -d STP5 -h 6+6+6+6 -i "$outer" -n 1 -a PEMBAE -p ../../../pdbs/stp5 -t ../../../temp -e "$value" -r "$value" -v info > ../../../data/stp5/STP5_"$outer"_PEMBAE_"$value".out
        ./embhs -d STP5 -h 6+6+6+6 -i "$outer" -n 1 -a AIDA -p ../../../pdbs/stp5 -t ../../../temp -e "$value" -r "$value" -v info > ../../../data/stp5/STP5_"$outer"_AIDA_"$value".out
    done
done
