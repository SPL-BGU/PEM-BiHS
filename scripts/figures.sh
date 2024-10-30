cd "$(dirname "$0")/.." || exit 1

mkdir -p results/figures

echo "Generating Expansions/Time and Ablation Figures"
python3 analysis/figures.py
echo "Generating Tables"
python3 analysis/tables.py
