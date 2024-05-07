from rais import Rais
from lyu import Lyu
from sampaio import Sampaio
from utils import get_instance_data

from pathlib import Path
from typing import Optional

import argparse
import re


def lyu(path: Path) -> None:
    """
    Solve the instance found at path with Lyu model, print the result
    :param path: file containing the instance data
    :return: Nothing
    """
    g, v, r = get_instance_data(path)

    model = Lyu(g, v, r)
    model.optimize()

    print(path.name, '\tLyu \t', model.get_result())


def sampaio(path: Path) -> None:
    """
    Solve the instance found at path with Sampaio model, print the result
    :param path: file containing the instance data
    :return: Nothing
    """
    g, v, r = get_instance_data(path, sampaio=True)

    model = Sampaio(g, v, r)
    model.optimize()

    print(path.name, '\tSampaio\t', model.get_result())


def rais(path: Path) -> None:
    """
    Solve the instance found at path with Rais model, print the result
    :param path: file containing the instance data
    :return: Nothing
    """
    g, v, r = get_instance_data(path)

    model = Rais(g, v, r)
    model.optimize()

    print(path.name, '\tRais\t', model.get_result())


def _get_pdpt_path(r: int, k: int, t: int, num: int) -> Optional[Path]:
    params = f'PDPT-R{r}-K{k}-T{t}'
    path = Path(f'../data/PDPT/{params}/{params}-Q100-{num}.txt')
    if path.exists():
        return path
    else:
        return None


def _get_pdptwt_path(r: int, k: int, t: int, shift: int, d: str, num: int) -> Optional[Path]:
    params = f'{r}R-{k}K-{t}T'
    path = Path(f'../data/PDPTWT/{params.replace('-', '')}/{params}-{shift}{d}-{num}.txt')
    if path.exists():
        return path
    else:
        return None


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Quickly test an instance with the specified model')
    parser.add_argument('instance', type=str, help='Instance to run, e.g. PDPT-R5-K2-T1-Q100-6')
    parser.add_argument('model', type=str, help='Model to use')

    args = parser.parse_args()

    model = args.model
    if model.lower() not in ['lyu', 'sampaio', 'rais']:
        print('Model must be either Rais, Lyu or Sampaio')
        exit(1)

    pdpt_instance = r'PDPT-R(\d+)-K(\d+)-T(\d+)-Q100-(\d+)'
    pdptwt_instance = r'(\d+)R-(\d+)K-(\d+)T-(\d+)(L|M|S)-(\d+)'

    instance = args.instance
    if instance.endswith('.txt'):
        instance = instance[:-4]

    path = None
    if re.match(pdpt_instance, instance):
        match_res = re.match(pdpt_instance, instance)
        path = _get_pdpt_path(match_res.group(1), match_res.group(2), match_res.group(3), match_res.group(4))
    elif re.match(pdptwt_instance, instance):
        match_res = re.match(pdptwt_instance, instance)
        path = _get_pdptwt_path(match_res.group(1), match_res.group(2), match_res.group(3), match_res.group(4),
                                match_res.group(5), match_res.group(6))

    if not path:
        print('Instance does not exist')
        exit(1)

    if model.lower() == 'rais' and 'PDPTWT' not in path.parts:
        rais(path)
    elif model.lower() == 'sampaio' and 'PDPTWT' in path.parts:
        sampaio(path)
    elif model.lower() == 'lyu':
        lyu(path)
    else:
        print(f'{model.title()} model cannot solve {instance}')
