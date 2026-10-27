// usage: node render.js scene.json views.json outprefix [W H]
const { chromium } = require('playwright');
const fs = require('fs');
const path = require('path');
(async () => {
  const [sceneFile, viewsFile, outPrefix] = process.argv.slice(2, 5);
  const W = parseInt(process.argv[5] || '1400'), H = parseInt(process.argv[6] || '1000');
  const scene = JSON.parse(fs.readFileSync(sceneFile, 'utf8'));
  const views = JSON.parse(fs.readFileSync(viewsFile, 'utf8'));
  const browser = await chromium.launch({ args: ['--use-angle=swiftshader', '--enable-unsafe-swiftshader', '--allow-file-access-from-files'] });
  const page = await browser.newPage({ viewport: { width: W, height: H } });
  page.on('console', m => { if (m.type() === 'error') console.log('console:', m.text()); });
  page.on('pageerror', e => console.log('pageerror:', e.message));
  await page.goto('file://' + path.resolve(__dirname, 'viewer.html'));
  await page.waitForFunction(() => window.ready === true, null, { timeout: 30000 });
  for (const v of views) {
    const url = await page.evaluate(async ([sc, vw, w, h]) => await window.renderScene(sc, vw, w, h), [scene, v, W, H]);
    const b64 = url.split(',')[1];
    const out = `${outPrefix}_${v.name}.png`;
    fs.writeFileSync(out, Buffer.from(b64, 'base64'));
    console.log('wrote', out);
  }
  await browser.close();
})();
