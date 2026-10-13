# GitHub Pages

После слияния PR откройте **Settings → Pages → Build and deployment** и выберите **Source: GitHub Actions**.

Workflow **Deploy to GitHub Pages** запускается на каждом push в main; при необходимости можно запустить вручную в **Actions → Deploy to GitHub Pages → Run workflow**.

Адрес симуляции: https://motostrelok1.github.io/boat-dock-ai/

Адрес диагностики: https://motostrelok1.github.io/boat-dock-ai/benchmark.html

Перед деплоем GitHub запускает npm ci, npm test и npm run build. В сборку входят обе HTML-страницы.
