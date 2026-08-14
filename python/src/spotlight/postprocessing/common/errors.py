class SpotlightDataCorruptionError(Exception):
    """Raised when on-disk recording data fails an integrity check (e.g. an
    empty metadata file) that a plain parse error wouldn't make obvious.
    """
