"""Flask routes, adapters, templates, and static assets for the SLVROV UI."""
from flask import Flask, jsonify, render_template

from .. import control_objects
from .routes.setup_routes.action_crud import action_crud_bp
from .routes.setup_routes.motor_crud import motor_crud_bp


PACKAGE_AREA = 'web'


def get_default_action_options():
    """Return predefined ROV action options for setup dashboards."""

    return [
        {
            "action_name": action.value[0],
            "action_type": str(action.value[1]),
        }
        for action in control_objects.DefaultROVActions
    ]


def create_app():
    app = Flask(__name__)

    app.register_blueprint(action_crud_bp)
    app.register_blueprint(motor_crud_bp)

    @app.route('/')
    def home():
        return jsonify({"msg": "home"})

    @app.route('/setup/actions')
    @app.route('/dashboard/actions')
    def action_dashboard():
        return render_template(
            'action_dashboard.html',
            default_actions=get_default_action_options(),
        )

    return app

app = create_app()
if __name__ == '__main__':
    app.run()
