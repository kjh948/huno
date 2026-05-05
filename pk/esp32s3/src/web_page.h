#ifndef WEB_PAGE_H
#define WEB_PAGE_H

const char* html_page = R"=====(
<!DOCTYPE html>
<html>
<head>
    <title>Huno Web Control</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <style>
        body { font-family: Arial, sans-serif; margin: 20px; background-color: #f0f2f5; }
        .control-group { margin-bottom: 15px; display: flex; align-items: center; }
        .btn { padding: 10px 20px; font-size: 16px; margin-right: 10px; cursor: pointer; background-color: #007bff; color: white; border: none; border-radius: 5px; }
        .btn:hover { background-color: #0056b3; }
        .btn-danger { background-color: #dc3545; }
        .btn-danger:hover { background-color: #c82333; }
        label { display: inline-block; width: 120px; font-weight: bold; }
        input[type=range] { width: 250px; margin-right: 15px; }
        .value { display: inline-block; width: 60px; text-align: right; background: #fff; padding: 5px; border-radius: 3px; border: 1px solid #ccc; }
        .container { max-width: 600px; margin: auto; background: white; padding: 20px; border-radius: 10px; box-shadow: 0 4px 6px rgba(0,0,0,0.1); }
    </style>
</head>
<body>
    <div class="container">
        <h1 style="text-align: center;">Huno Web Control</h1>
        <div class="control-group" style="justify-content: center; margin-bottom: 30px;">
            <button class="btn" onclick="sendCommand('w')">Start Walk</button>
            <button class="btn btn-danger" onclick="sendCommand('s')">Stop Walk</button>
        </div>
        <h2>Parameters</h2>
        <div id="params"></div>
    </div>

    <script>
        const paramsConfig = [
            { id: 'f', name: 'Frequency', min: 0.1, max: 10.0, step: 0.1 },
            { id: 'h', name: 'Height', min: 0, max: 300, step: 1 },
            { id: 'y', name: 'Shift Y', min: -100, max: 100, step: 1 },
            { id: 'sh', name: 'Step Height', min: 0, max: 100, step: 1 },
            { id: 'sl', name: 'Step Length', min: -100, max: 100, step: 1 },
            { id: 'ss', name: 'Side Step', min: -100, max: 100, step: 1 },
            { id: 'as', name: 'Arm Swing', min: 0, max: 100, step: 1 },
            { id: 'd', name: 'Direction', min: -2, max: 2, step: 0.1 },
            { id: 'sc', name: 'Scale', min: 0.1, max: 5.0, step: 0.1 }
        ];

        function createSliders() {
            const container = document.getElementById('params');
            paramsConfig.forEach(p => {
                const div = document.createElement('div');
                div.className = 'control-group';
                div.innerHTML = `
                    <label>${p.name}</label>
                    <input type="range" id="${p.id}" min="${p.min}" max="${p.max}" step="${p.step}" onchange="updateParam('${p.id}')" oninput="document.getElementById('${p.id}_val').innerText=this.value">
                    <span class="value" id="${p.id}_val">0</span>
                `;
                container.appendChild(div);
            });
        }

        function updateParam(id) {
            const val = document.getElementById(id).value;
            fetch(`/set?param=${id}&value=${val}`);
        }

        function sendCommand(cmd) {
            fetch(`/cmd?action=${cmd}`);
        }

        function fetchParams() {
            fetch('/getParams')
                .then(r => r.json())
                .then(data => {
                    paramsConfig.forEach(p => {
                        if (data[p.id] !== undefined) {
                            const val = data[p.id];
                            document.getElementById(p.id).value = val;
                            document.getElementById(p.id + '_val').innerText = val;
                        }
                    });
                })
                .catch(err => console.error(err));
        }

        createSliders();
        fetchParams();
        setInterval(fetchParams, 5000); // Poll every 5s to refresh UI if changed elsewhere
    </script>
</body>
</html>
)=====";

#endif
