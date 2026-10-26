import sys
from playwright.sync_api import sync_playwright
with sync_playwright() as p:
    b=p.chromium.launch(executable_path='/home/claud/.cache/ms-playwright/chromium_headless_shell-1243/chrome-headless-shell-linux64/chrome-headless-shell')
    for w,scheme,name in [(1000,'light','l'),(390,'dark','m')]:
        pg=b.new_page(viewport={'width':w,'height':900},color_scheme=scheme)
        pg.goto('file:///workspace/agents/worker-5/t0877/report/index.html')
        pg.screenshot(path=f'/tmp/claude-1000/-workspace-agents-worker-5/477fa8a4-f111-4c55-ab73-c97d16ac49a5/scratchpad/shot_{name}.png',full_page=True)
        print(name, pg.evaluate('document.documentElement.scrollWidth'))
    b.close()
