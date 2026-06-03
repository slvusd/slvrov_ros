"""Flask routes, adapters, templates, and static assets for the SLVROV UI."""
from flask import Flask  # type: ignore
from .routes.setup_routes import *


PACKAGE_AREA = 'web'


def create_app():
    app = Flask(__name__)

    ... # register blueprints, setup routes, etc.

    return app