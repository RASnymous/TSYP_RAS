import os

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))


def get_package_share_directory(name):
    return ROOT          # the package source folder has the same layout (models/, worlds/...)
