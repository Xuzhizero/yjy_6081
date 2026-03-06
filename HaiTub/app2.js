// server.js
const express = require('express');
const fs = require('fs');
const path = require('path');

const app = express();
const PORT = 3000;

app.get('/style.json', (req, res) => {
    const filePath = path.join(__dirname, 'style.json');
    const stat = fs.statSync(filePath);
    res.setHeader('Content-Type', 'application/json');
    res.setHeader('Cache-Control', 'public, max-age=86400'); // 可选
    res.setHeader('Content-Length', stat.size);
    fs.createReadStream(filePath).pipe(res);
});

app.listen(PORT, () => console.log(`Server running on http://localhost:${PORT}`));
