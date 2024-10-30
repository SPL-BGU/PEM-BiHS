import pandas as pd


def toh_pivot(df):
    agg_df = df.groupby(['domain', 'heuristic', 'alg', 'threads']).agg(
        avg_expanded=('expanded', 'mean'),
        avg_elapsed=('elapsed', 'mean')
    ).reset_index()

    agg_df['avg_nps'] = agg_df['avg_expanded'] / agg_df['avg_elapsed']

    print(agg_df)


def stp4_pivot(df):
    agg_df = df.groupby(['domain', 'heuristic', 'alg', 'threads']).agg(
        avg_expanded=('expanded', 'mean'),
        avg_elapsed=('elapsed', 'mean')
    ).reset_index()

    pivot_df = agg_df.pivot_table(
        index=['alg', 'threads'],
        columns='heuristic',
        values=['avg_expanded', 'avg_elapsed'],
        aggfunc='mean'
    )

    # Flatten the MultiIndex columns
    pivot_df.columns = [f"{heuristic}_{stat}" for stat, heuristic in pivot_df.columns]

    # Rename the columns replacing '4+4+4+4' with 'PDB'
    pivot_df.columns = [col.replace('4+4+4+4', 'PDB') for col in pivot_df.columns]

    pivot_df['MD_avg_nps'] = pivot_df['MD_avg_expanded'] / pivot_df['MD_avg_elapsed']
    pivot_df['PDB_avg_nps'] = pivot_df['PDB_avg_expanded'] / pivot_df['PDB_avg_elapsed']

    # Define the desired column order
    desired_order = [
        'MD_avg_elapsed',
        'MD_avg_expanded',
        'MD_avg_nps',
        'PDB_avg_elapsed',
        'PDB_avg_expanded',
        'PDB_avg_nps',
    ]

    # Add any remaining columns that are not in the desired order
    remaining_columns = [col for col in pivot_df.columns if col not in desired_order]

    # Concatenate the desired order with remaining columns
    new_column_order = desired_order + remaining_columns

    # Reorder the DataFrame columns
    pivot_df = pivot_df[new_column_order]

    pd.options.display.float_format = '{:,.2f}'.format
    print(pivot_df)


def main():
    stp4_df = pd.read_csv('results/csvs/stp4.csv')
    condition = ~((stp4_df['alg'].str.startswith('PEM-BAE')) | (stp4_df['alg'].str.startswith('AIDA'))) | (
            stp4_df['threads'] == 96)
    stp4_df = stp4_df[condition]

    print("STP4 All Instances Table".center(90, '-'))
    print()
    stp4_pivot(stp4_df)

    hard_stp4_df = stp4_df[stp4_df['instance_id'].isin([2, 14, 16, 31, 48, 55, 59, 65, 81, 87])]
    print()
    print("STP4 Hard Instances Table".center(90, '-'))
    print()
    stp4_pivot(hard_stp4_df)

    toh_df = pd.read_csv('results/csvs/toh.csv')
    condition = (~toh_df['alg'].str.startswith('PEM-BAE')) | (toh_df['threads'] == 96)
    toh_df = toh_df[condition]
    print()
    print("TOH Table".center(90, '-'))
    print()
    toh_pivot(toh_df)


if __name__ == '__main__':
    main()
