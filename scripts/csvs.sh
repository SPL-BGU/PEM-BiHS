cd "$(dirname "$0")/.." || exit 1

mkdir -p results/csvs

echo "Processing stp4 logs"
python3 analysis/logs_to_csv.py -l data/stp4 -o results/csvs/stp4.csv
echo "Processing stp5 logs"
python3 analysis/logs_to_csv.py -l data/stp5 -o results/csvs/stp5.csv
echo "Processing toh logs"
python3 analysis/logs_to_csv.py -l data/toh -o results/csvs/toh.csv
