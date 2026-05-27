import importlib


def test_slvrov_web_ui_imports():
    package = importlib.import_module('slvrov_web_ui')

    assert package.PACKAGE_NAME == 'slvrov_web_ui'


def test_slvrov_web_ui_placeholder_packages_import():
    importlib.import_module('slvrov_web_ui.nodes')
    importlib.import_module('slvrov_web_ui.routes')
    importlib.import_module('slvrov_web_ui.ros_adapters')
