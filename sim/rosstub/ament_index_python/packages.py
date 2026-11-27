import os
class PackageNotFoundError(LookupError): pass
def get_package_share_directory(name):
    if name.startswith('apnvi_stage'):
        n = name[-1]
        root = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', '..')
        return os.path.normpath(os.path.join(root, 'src', name))
    raise PackageNotFoundError(name)
