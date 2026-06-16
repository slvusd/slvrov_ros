"""Flask routes, adapters, templates, and static assets for the SLVROV UI."""
from flask import Flask, jsonify
from .routes.setup_routes.action_crud import action_crud_bp
from .routes.setup_routes.motor_crud import motor_crud_bp


PACKAGE_AREA = 'web'


def create_app():
    app = Flask(__name__)

    app.register_blueprint(action_crud_bp)
    app.register_blueprint(motor_crud_bp)

    @app.route('/')
    def home():
        return jsonify({"msg": "home"})

    return app

app = create_app()
if __name__ == '__main__':
    app.run()
