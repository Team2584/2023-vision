from flask import Flask, render_template, request, abort
from werkzeug.middleware.proxy_fix import ProxyFix

app = Flask(__name__)

app.wsgi_app = ProxyFix(
        app.wsgi_app, x_for = 1, x_proto = 1, x_host = 1, x_prefix = 1)

@app.route("/<page>")
def cones_or_cubes(page):
    if not page == "cone" and not page == "cube":
        abort(404)

    filename = page + "-params.txt"

    with open(filename, "r") as f:
        params = f.readlines();

    html_page = page + ".html"
    return render_template(html_page, hue_min = params[0],
                                      hue_max = params[1],
                                      sat_min = params[2],
                                      sat_max = params[3],
                                      val_min = params[4],
                                      val_max = params[5])

@app.route("/send-cones", methods=['POST'])
def send_cones():
    data = request.json
    with open("cone-params.txt", "w") as f:
        for item in data.values():
            f.write(item)
            f.write("\n")
    return {'did': True}

@app.route("/send-cubes", methods=['POST'])
def send_cubes():
    data = request.json
    with open("cube-params.txt", "w") as f:
        for item in data.values():
            f.write(item)
            f.write("\n")
    return {'did': True}

if __name__ == "__main__":
    app.run()
