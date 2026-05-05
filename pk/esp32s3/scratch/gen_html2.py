html = """<!DOCTYPE html>
<html>
<head>
    <title>HUNO Console</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <style>
        body { font-family: monospace; background: #1a1a1a; color: #00ff00; margin: 0; display: flex; flex-direction: column; height: 100vh; overflow: hidden; }
        #log { flex: 1; overflow-y: auto; padding: 10px; white-space: pre-wrap; font-size: 14px; border-bottom: 1px solid #333; }
        #input-container { display: flex; padding: 10px; background: #222; }
        #input { flex: 1; background: #000; color: #00ff00; border: 1px solid #444; padding: 10px; font-family: inherit; outline: none; }
        #send-btn { background: #00ff00; color: #000; border: none; padding: 10px 20px; margin-left: 10px; cursor: pointer; font-weight: bold; border-radius: 4px; }
        #send-btn:active { background: #00cc00; }
    </style>
</head>
<body>
    <div id="log"></div>
    <div id="input-container">
        <input type="text" id="input" placeholder="Type command..." autofocus autocomplete="off">
        <button id="send-btn">SEND</button>
    </div>
    <script>
        var log = document.getElementById('log');
        var input = document.getElementById('input');
        var btn = document.getElementById('send-btn');
        var ws = new WebSocket((window.location.protocol === 'https:' ? 'wss://' : 'ws://') + window.location.host + window.location.pathname + 'ws');
        
        ws.onmessage = function(e) {
            log.textContent += e.data;
            log.scrollTop = log.scrollHeight;
        };
        
        function send() {
            var msg = input.value;
            if (msg.length > 0) {
                ws.send(msg);
                input.value = '';
            }
            input.focus();
        }
        
        btn.onclick = send;
        input.onkeydown = function(e) { if (e.keyCode === 13) send(); };
        ws.onopen = function() { log.textContent += "[Connected to HUNO]\\n"; };
        ws.onclose = function() { log.textContent += "\\n[Disconnected]"; };
        
        document.body.onclick = function() { input.focus(); };
    </script>
</body>
</html>"""

hex_data = ','.join(str(b) for b in html.encode('utf-8'))

print(f"const uint32_t WEBSERIAL_HTML_SIZE = {len(html)};")
print(f"const uint8_t WEBSERIAL_HTML[] PROGMEM = {{ {hex_data} }};")
