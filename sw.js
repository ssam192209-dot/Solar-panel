// Service worker sederhana untuk dashboard Solar Tracker.
// Menyimpan cache app-shell (HTML/CSS/JS/library) supaya halaman tetap bisa
// dibuka walau koneksi sempat putus. Data Firebase TIDAK di-cache (harus selalu live).

const CACHE_NAME = 'solar-tracker-cache-v1';
const APP_SHELL = ['./solar-tracker-dashboard.html', './manifest.json'];

self.addEventListener('install', (event) => {
  event.waitUntil(
    caches.open(CACHE_NAME).then((cache) => cache.addAll(APP_SHELL))
  );
  self.skipWaiting();
});

self.addEventListener('activate', (event) => {
  event.waitUntil(
    caches.keys().then((keys) =>
      Promise.all(keys.filter((k) => k !== CACHE_NAME).map((k) => caches.delete(k)))
    )
  );
  self.clients.claim();
});

self.addEventListener('fetch', (event) => {
  const url = event.request.url;

  // Jangan sentuh koneksi realtime/auth Firebase -- itu harus selalu live, tidak boleh dari cache.
  if (url.includes('firebaseio.com') || url.includes('googleapis.com') || url.includes('firebaseapp.com')) {
    return;
  }

  event.respondWith(
    caches.match(event.request).then((cached) => {
      const networkFetch = fetch(event.request)
        .then((response) => {
          if (response && response.status === 200) {
            const clone = response.clone();
            caches.open(CACHE_NAME).then((cache) => cache.put(event.request, clone));
          }
          return response;
        })
        .catch(() => cached);
      return cached || networkFetch;
    })
  );
});
