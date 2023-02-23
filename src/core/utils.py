import psutil


def verbose_print(verbose: bool):
    if verbose:

        def _vprint(*args, **kwargs):
            print(*args, **kwargs)

    else:
        _vprint = lambda *_, **__: None  # do-nothing function
    return _vprint


def available_cpu_count():
    return psutil.cpu_count(logical=False)
