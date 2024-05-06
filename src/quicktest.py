from src.models.rais import Rais
from src.models.lyu import Lyu
from src.models.sampaio import Sampaio
from src.utils import get_instance_data

from pathlib import Path


def lyu(path: Path) -> None:
    g, v, r = get_instance_data(path)

    model = Lyu(g, v, r)
    model.optimize()

    print(path.name, '\tLyu \t', model.get_result())


def sampaio(path: Path, *, vi: bool = False) -> None:
    g, v, r = get_instance_data(path, sampaio=not vi)

    model = Sampaio(g, v, r, vi=vi)
    model.optimize()

    print(path.name, '\tSampaio\t', model.get_result())


def rais(path: Path) -> None:
    g, v, r = get_instance_data(path)

    model = Rais(g, v, r)
    model.optimize()

    print(path.name, '\tRais\t', model.get_result())


def get_pdpt_path(r: int, k: int, t: int, num: int) -> Path:
    params = f'PDPT-R{r}-K{k}-T{t}'
    return Path(f'../data/PDPT/{params}/{params}-Q100-{num}.txt')


def get_pdptwt_path(r: int, k: int, t: int, shift: int, d: str, num: int) -> Path:
    params = f'{r}R-{k}K-{t}T'
    return Path(f'../data/PDPTWT/{params.replace('-', '')}/{params}-{shift}{d}-{num}.txt')


if __name__ == '__main__':
    p0 = get_pdpt_path(5, 2, 1, 6)
    # p1 = get_pdptwt_path(3, 4, 4, 180, 'M', 0)
    rais(p0)
    lyu(p0)
    # sampaio(p1, vi=False)
    # sampaio(p1, vi=True)
    # lyu(p1)
