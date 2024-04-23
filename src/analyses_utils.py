import pandas as pd

_OPT = ['OPTIMAL']
_TIME = ['TIME_LIMIT', 'FEASIBLE']
_NO = ['TIME_LIMIT', 'TIMEOUT']


def get_results_data(df: pd.DataFrame) -> pd.DataFrame:
    """
    Given a DataFrame containing a computation's results, returns data on performance.
    This function expects a DataFrame containing the columns Params, Time, Gap, Status
    """
    df_opt = _get_n_opt(df)
    df_lim = _get_n_lim(df)
    df_no = _get_n_no(df)
    df_gap = _get_gap(df)
    df_t = _get_t(df)

    result = pd.merge(df_opt, df_lim, how='outer', left_index=True, right_index=True)
    result = pd.merge(result, df_no, how='outer', left_index=True, right_index=True)
    result = pd.merge(result, df_gap, how='outer', left_index=True, right_index=True)
    result = pd.merge(result, df_t, how='outer', left_index=True, right_index=True)

    result.fillna(0, inplace=True)
    result['opt'] = result['opt'].astype(int)
    result['lim'] = result['lim'].astype(int)
    result['no'] = result['no'].astype(int)

    return result


def _get_n_opt(df: pd.DataFrame) -> pd.DataFrame:
    res = df[df['Status'].isin(_OPT)].groupby('Params').count()
    res['Instance'].name = 'opt'
    return res['Instance']


def _get_n_lim(df: pd.DataFrame) -> pd.DataFrame:
    res = df[(df['Status'].isin(_TIME)) & (df['Objective'] != float('inf'))].groupby('Params').nunique()
    res['Instance'].name = 'lim'
    return res['Instance']


def _get_n_no(df: pd.DataFrame) -> pd.DataFrame:
    res = df[(df['Status'].isin(_NO)) & (df['Objective'] == float('inf'))].groupby('Params').nunique()
    res['Instance'].name = 'no'
    return res['Instance']


def _get_gap(df: pd.DataFrame) -> pd.DataFrame:
    res = df[(df['Status'].isin(_TIME)) & (df['Objective'] != float('inf'))].groupby('Params')['Gap'].mean()
    res.name = 'gap'
    return res


def _get_t(df: pd.DataFrame) -> pd.DataFrame:
    res = df[df['Status'].isin(_OPT)].groupby('Params')['Time'].mean()
    res.name = 't'
    return res


if __name__ == '__main__':
    df = pd.read_csv('../results/PDPT.csv')
    df['Params'] = df['Instance'].apply(lambda name: name[5:13])

    df_rais = df[df['Model'] == 'Rais']
    df_lyu = df[df['Model'] == 'Lyu']

    print('Rais')
    t = get_results_data(df_rais)
    print(t)

    print('Lyu')
    t = get_results_data(df_lyu)
    print(t)
