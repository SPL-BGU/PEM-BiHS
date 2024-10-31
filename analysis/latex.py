import pandas as pd


def generate_table(df, file_path):
    # Sort by instance_id, then by alg based on specified order
    # Define the algorithm order
    alg_order = {v: i for i, v in enumerate(["PEM-BAE*", "PEMM", "PEM-A*", "PEM-rA*", "AIDA*", "RAIDA*"])}

    df['alg_order'] = df['alg'].map(alg_order)
    # Sort the DataFrame by instance and then algorithm
    df = df.sort_values(by=['instance_id', 'alg_order']).reset_index(drop=True)
    df = df.drop(columns='alg_order')

    # Round `elapsed` and format `expanded` with thousand separator
    df['elapsed'] = df['elapsed'].round().astype(int).apply(lambda x: f"{x:,}")
    df['expanded'] = df['expanded'].apply(lambda x: f"{x:,}")
    df['generated'] = df['generated'].apply(lambda x: f"{x:,}")

    # Construct the LaTeX table string
    latex_string = "\\begin{tabular}{|l|l|r|r|r|r|}\n\\hline\n"
    latex_string += (
        "\\multicolumn{1}{|c|}{\\textbf{Instance}} & "
        "\\multicolumn{1}{c|}{\\textbf{Algorithm}} & "
        "\\multicolumn{1}{c|}{\\textbf{Expanded}} & "
        "\\multicolumn{1}{c|}{\\textbf{Generated}} & "
        "\\multicolumn{1}{c|}{\\textbf{Elapsed}} & "
        "\\multicolumn{1}{c|}{\\textbf{Solution}} \\\\\n\\hline\n"
    )

    current_instance_id = None
    for _, row in df.iterrows():
        if row['instance_id'] != current_instance_id:
            if current_instance_id is not None:
                latex_string += "\\hline\n"  # Add line after each instance_id cluster
            current_instance_id = row['instance_id']

        alg = row['alg'].replace('R','r')
        alg = alg if alg!='PEMM' else 'PEM-MM'
        latex_string += f"{row['instance_id']} & {alg} & {row['expanded']} & {row['generated']} & {row['elapsed']} & {row['solution']} \\\\\n"

    latex_string += "\\hline\n\\end{tabular}"

    # Save to a .tex file
    with open(file_path, "w") as file:
        file.write(latex_string)


def main():
    stp5_df = pd.read_csv('results/csvs/stp5.csv')
    stp5_df = stp5_df[stp5_df['threads'] == 96]
    generate_table(stp5_df[stp5_df['instance_id'] < 20].copy(), 'results/latex/stp5_first_table.txt')
    generate_table(stp5_df[stp5_df['instance_id'] >= 20].copy(), 'results/latex/stp5_second_table.txt')

    toh_df = pd.read_csv('results/csvs/toh.csv')
    generate_table(toh_df, 'results/latex/toh_table.txt')




if __name__ == '__main__':
    main()
