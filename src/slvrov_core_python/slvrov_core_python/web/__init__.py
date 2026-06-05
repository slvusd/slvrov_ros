"""Flask routes, adapters, templates, and static assets for the SLVROV UI."""
from flask import Flask, jsonify  # type: ignore
from routes.setup_routes.custom_actions import *


PACKAGE_AREA = 'web'


def create_app():
    app = Flask(__name__)

    app.register_blueprint(custom_actions_bp, url_prefix="/custom_actions")

    @app.route('/')
    def home():
        return jsonify({"msg": "home"})

    return app

app = create_app()
if __name__ == '__main__':
    app.run()