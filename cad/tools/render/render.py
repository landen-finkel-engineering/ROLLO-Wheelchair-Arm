#!/usr/bin/env python3
"""Headless shaded renderer for STL scenes (three.js in Playwright Chromium).

Usage (python API):
    from render import render_scene
    render_scene(parts=[{"file": "a.stl", "color": "#cc8844", "opacity": 1.0}], out="x.png",
                 view="iso", size=(1400, 1000), title="...")

Scene coordinates: millimetres, Z up (CAD convention). The page converts to three.js Y-up.
"""
import json, os, sys, threading, http.server, socketserver, functools, tempfile, shutil, pathlib

HERE = pathlib.Path(__file__).resolve().parent

PAGE = r"""<!doctype html><html><head><meta charset="utf-8">
<style>html,body{margin:0;background:#ffffff;overflow:hidden}#t{position:absolute;left:16px;top:10px;font:600 20px/1.3 Helvetica,Arial,sans-serif;color:#222}
#s{position:absolute;left:16px;bottom:10px;font:13px/1.3 Helvetica,Arial,sans-serif;color:#555}</style>
<script type="importmap">{"imports":{"three":"/node_modules/three/build/three.module.js","three/addons/":"/node_modules/three/examples/jsm/"}}</script>
</head><body><div id="t"></div><div id="s"></div>
<script type="module">
import * as THREE from 'three';
import {STLLoader} from 'three/addons/loaders/STLLoader.js';
const scene_desc = await (await fetch('/__scene.json')).json();
document.getElementById('t').textContent = scene_desc.title || '';
document.getElementById('s').textContent = scene_desc.subtitle || '';
const W = scene_desc.size[0], H = scene_desc.size[1];
const renderer = new THREE.WebGLRenderer({antialias:true, preserveDrawingBuffer:true});
renderer.setPixelRatio(1); renderer.setSize(W, H);
renderer.setClearColor(0xffffff, 1);
document.body.appendChild(renderer.domElement);
const scene = new THREE.Scene();
const root = new THREE.Group();
root.rotation.x = -Math.PI/2;   // CAD Z-up -> three Y-up
scene.add(root);
scene.add(new THREE.HemisphereLight(0xffffff, 0x8d8d8d, 1.6));
const d1 = new THREE.DirectionalLight(0xffffff, 1.6); d1.position.set(1, 2, 1.5); scene.add(d1);
const d2 = new THREE.DirectionalLight(0xffffff, 0.6); d2.position.set(-1.5, 0.5, -1); scene.add(d2);
const loader = new STLLoader();
const box = new THREE.Box3();
for (const p of scene_desc.parts) {
  const geo = await new Promise((res, rej) => loader.load(p.url, res, undefined, rej));
  geo.computeVertexNormals();
  const mat = new THREE.MeshStandardMaterial({color: p.color || '#bbbbbb', metalness: p.metal ? 0.6 : 0.05,
       roughness: p.metal ? 0.35 : 0.6, transparent: (p.opacity ?? 1) < 1, opacity: p.opacity ?? 1,
       side: THREE.DoubleSide, flatShading: false});
  const m = new THREE.Mesh(geo, mat);
  root.add(m);
  if (p.edges) {
    const eg = new THREE.EdgesGeometry(geo, 30);
    const lines = new THREE.LineSegments(eg, new THREE.LineBasicMaterial({color: 0x333333, transparent:true, opacity:0.35}));
    root.add(lines);
  }
}
root.updateMatrixWorld(true);
box.setFromObject(root);
const c = new THREE.Vector3(); box.getCenter(c);
const sz = new THREE.Vector3(); box.getSize(sz);
const r = sz.length() / 2;
// camera directions given in CAD coordinates (X fwd, Y left, Z up); converted to three (x, z, -y)
const views = {iso:[1,1,0.75], iso_right:[1,-1,0.75], iso_back:[-1,1,0.75], iso_back_right:[-1,-1,0.75],
               front:[1,0,0], back:[-1,0,0], left:[0,1,0], right:[0,-1,0], top:[0,0,1], bottom:[0,0,-1],
               low_left:[1,1.2,0.25], low_right:[1,-1.2,0.25]};
let v = scene_desc.view_vec || views[scene_desc.view] || views.iso;
const dir = new THREE.Vector3(v[0], v[2], -v[1]).normalize();
const aspect = W/H;
const cam = new THREE.OrthographicCamera(-1,1,1,-1,-1e5,1e5);
const zoom = scene_desc.zoom || 1.0;
const half = r * 1.08 / zoom;
cam.left = -half*aspect; cam.right = half*aspect; cam.top = half; cam.bottom = -half;
cam.up.set(0,1,0);
if (Math.abs(dir.y) > 0.99) cam.up.set(0,0,-1);
cam.position.copy(c.clone().add(dir.clone().multiplyScalar(r*4)));
cam.lookAt(c);
cam.updateProjectionMatrix();
renderer.render(scene, cam);
window.__done = true;
</script></body></html>"""

def _node_modules() -> pathlib.Path:
    """three.js location: ./node_modules (run `npm install` here) or $WRA_THREE_NODE_MODULES."""
    env = os.environ.get("WRA_THREE_NODE_MODULES")
    for c in ([pathlib.Path(env)] if env else []) + [HERE / "node_modules", pathlib.Path("/home/claude/render/node_modules")]:
        if (c / "three" / "build" / "three.module.js").exists():
            return c
    raise FileNotFoundError("three.js not found: run `npm install` in cad/tools/render (needs Node.js)")


def available() -> bool:
    try:
        import playwright  # noqa: F401
        _node_modules()
        return True
    except Exception:
        return False


class _Quiet(http.server.SimpleHTTPRequestHandler):
    def log_message(self, *a):
        pass

def render_scene(parts, out, view="iso", size=(1400, 1000), title="", subtitle="", zoom=1.0, view_vec=None):
    from playwright.sync_api import sync_playwright
    work = pathlib.Path(tempfile.mkdtemp(prefix="rnd_"))
    try:
        (work / "index.html").write_text(PAGE)
        # copy only the two three.js files the page needs (works on Windows without symlinks)
        nm = _node_modules()
        for rel in ("three/build/three.module.js", "three/examples/jsm/loaders/STLLoader.js"):
            dst = work / "node_modules" / rel
            dst.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy(nm / rel, dst)
        plist = []
        for i, p in enumerate(parts):
            dst = work / f"p{i}.stl"
            shutil.copy(p["file"], dst)
            q = dict(p); q.pop("file"); q["url"] = f"/p{i}.stl"
            plist.append(q)
        desc = {"parts": plist, "view": view, "size": list(size), "title": title, "subtitle": subtitle, "zoom": zoom}
        if view_vec is not None:
            desc["view_vec"] = list(view_vec)
        (work / "__scene.json").write_text(json.dumps(desc))
        handler = functools.partial(_Quiet, directory=str(work))
        with socketserver.TCPServer(("127.0.0.1", 0), handler) as httpd:
            port = httpd.server_address[1]
            th = threading.Thread(target=httpd.serve_forever, daemon=True); th.start()
            with sync_playwright() as pw:
                b = pw.chromium.launch(args=["--use-gl=angle", "--use-angle=swiftshader", "--enable-unsafe-swiftshader"])
                pg = b.new_page(viewport={"width": size[0], "height": size[1]})
                msgs = []
                pg.on("console", lambda m: msgs.append(m.text))
                pg.on("pageerror", lambda e: msgs.append(str(e)))
                pg.goto(f"http://127.0.0.1:{port}/index.html")
                try:
                    pg.wait_for_function("window.__done === true", timeout=180000)
                except Exception:
                    print("RENDER ERROR", msgs, file=sys.stderr)
                    raise
                pg.screenshot(path=str(out))
                b.close()
            httpd.shutdown()
    finally:
        shutil.rmtree(work, ignore_errors=True)
    return out

if __name__ == "__main__":
    spec = json.load(open(sys.argv[1]))
    render_scene(**spec)
