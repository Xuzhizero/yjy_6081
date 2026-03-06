const CACHE_NAME = "map-cache-v1";
const FILES_TO_CACHE = [
  "/style1.json",

];

self.addEventListener("install", event => {
  event.waitUntil(
    caches.open(CACHE_NAME).then(cache => cache.addAll(FILES_TO_CACHE))
  );
  self.skipWaiting(); // 激活新 SW
});

self.addEventListener("fetch", event => {
  event.respondWith(
    caches.match(event.request).then(cachedRes => {
      if (cachedRes) return cachedRes; // 如果缓存命中
      return fetch(event.request).then(networkRes => {
        // 可选：把网络结果放入缓存
        return caches.open(CACHE_NAME).then(cache => {
          cache.put(event.request, networkRes.clone());
          return networkRes;
        });
      });
    })
  );
});
