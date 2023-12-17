window.globalFrameCounter = Math.floor(Math.random()*120);

const N_PORTS = 8;
const hosts = [];

const randomMACBlock = ((m) => Math.floor(Math.random() * (m || 256)));
const randomMAC = (() => [ randomMACBlock(255), randomMACBlock(), randomMACBlock(), randomMACBlock(), randomMACBlock(), randomMACBlock() ]);
const macBlockToString = ((v) => v.toString(16).padStart(2,'0'));
const macToString = ((m) => m.reduce(((o,b) => o+(o&&':')+macBlockToString(b)), ''));

const makeElm = ((tagName, parentElm, className) => { const e = document.createElement(tagName); parentElm.appendChild(e); if (className) e.className = className; return e; });
const sleep = ((ms) => new Promise((r) => window.setTimeout(r,ms)));

/* error handling */
const frameworkError = ((e) =>
{
    console.error(e);
    document.body.classList.add('error');
    document.getElementById('error-message').innerText = (''+e);
});

/* render visualization */
const visContainer = document.getElementById('framework-visualization');
for (let i=0; i<N_PORTS; ++i)
{
    const host = {
        address: randomMAC(),
        sendQueue: [],
    };

    const hostOffsetX = (((2*i)+1)/(2*N_PORTS))*100;
    const switchOffsetX = 20+(((2*i)+1)/(2*N_PORTS))*60;

    const switchport = makeElm('div', visContainer, 'switchport');
    const wire = makeElm('div', visContainer, 'wire');
    const hostContainer = makeElm('div', visContainer, 'host');
    makeElm('div', wire, 'wire-1');
    makeElm('div', wire, 'wire-2');
    makeElm('div', wire, 'wire-3');
    if (i < 4)
    {
        wire.classList.add('left');
        wire.style.left = hostOffsetX.toFixed(2)+'%';
        wire.style.right = (100-switchOffsetX).toFixed(2)+'%';
    }
    else
    {
        wire.classList.add('right');
        wire.style.right = (100-hostOffsetX).toFixed(2)+'%';
        wire.style.left = switchOffsetX.toFixed(2)+'%';
    }

    const center = (N_PORTS-1)/2;
    const relDistance = Math.abs(i-center)/center;
    wire.style.top = ((35.5-15*relDistance)/.9)+'%';
    wire.style.bottom = 'min(12vh, 8vw)';
    host.wire = wire;

    switchport.style.top = ((35-15*relDistance)/.9)+'%';
    switchport.style.left = switchOffsetX.toFixed(2)+'%';

    makeElm('div', hostContainer, 'host-address').innerText = macToString(host.address);
    hostContainer.style.left = hostOffsetX.toFixed(2)+'%';

    hosts.push(host);
}
hosts.push({address: [0xff,0xff,0xff,0xff,0xff,0xff]});

const enqueueFrame = ((frame, port, outgoing) =>
{
    const frameNumber = new Uint32Array(new Uint8Array(frame.data).buffer);
    const host = hosts[port];
    const visual = makeElm('div', host.wire, outgoing ? 'frame out step-1' : 'frame in step-7');
    visual.style.backgroundColor = 'hsl('+frameNumber*(360*10/36)+',100%,50%)';
    visual.frameData = frame;
    visual.frameStep = outgoing ? 1 : 7;

    for (let i=0; i<=N_PORTS; ++i)
    {
        if (macToString(hosts[i].address) === macToString(frame.target))
        {
            makeElm('div', visual, 'label').innerText = '@'+((i === N_PORTS) ? 'B' : (''+i).padStart(2,'0'));
            break;
        }
    }
});

/* update loop */
(async () =>
{
    const heloResp = await fetch('/helo', { method:'POST' });
    if (!heloResp.ok)
    {
        frameworkError('Failed to start up');
        return;
    }
    
    const switchId = heloResp.headers.get('X-Switch-ID');
    while (true)
    {
        const nextTick = sleep(350); /* tick rate */
        try
        {
            let frames = [];
            for (let i=0; i < N_PORTS; ++i)
            {
                const host = hosts[i];
                
                let vlanUpdate = null;
                const targetVLAN = document.getElementById('vlan-update-'+i).value;
                if (targetVLAN !== document.getElementById('vlan-status-'+i).innerText)
                    vlanUpdate = parseInt(targetVLAN);
                
                let impacting = null;
                for (const frame of host.wire.querySelectorAll('.frame.in'))
                {
                    if (frame.frameStep === 1)
                    {
                        frame.frameStep = 0;
                        frame.classList.add('complete');

                        const targetingMe = [macToString(host.address), macToString(hosts[N_PORTS].address)].includes(macToString(frame.frameData.target));
                        if (frame.frameData.checksumOK && targetingMe)
                        {
                            frame.classList.add('ok');
                            frame.querySelector('.label').innerText = '✔\uFE0E';
                        }
                        else
                            frame.querySelector('.label').innerText = '✘\uFE0E';
                    }
                    else if (frame.frameStep === 0)
                    {
                        host.wire.removeChild(frame);
                    }
                    else
                    {
                        frame.classList.remove('step-'+frame.frameStep);
                        frame.classList.add('step-'+(--frame.frameStep));
                    }
                }
                for (const frame of host.wire.querySelectorAll('.frame.out'))
                {
                    if (frame.frameStep === 7)
                    {
                        host.wire.removeChild(frame);
                        impacting = frame.frameData;
                    }
                    else
                    {
                        frame.classList.remove('step-'+frame.frameStep);
                        frame.classList.add('step-'+(++frame.frameStep));
                    }
                }
                frames.push({ vlan: vlanUpdate, frame: impacting });

                if (host.sendQueue.length)
                    enqueueFrame(host.sendQueue.pop(), i, true);
            }

            const resp = await fetch('/tick', {
                method: 'POST',
                body: JSON.stringify(frames),
                headers: { 'X-Switch-ID': switchId },
            });
            if (!resp.ok)
            {
                if (resp.status === 412)
                    throw 'Connection superseded. Did you open the simulation in another tab?';
                else
                    throw ('HTTP/1.1 '+resp.status+' '+resp.statusText);
            }

            const incomingFrames = await resp.json();
            for (let i = 0; i < N_PORTS; ++i)
            {
                const { vlan, frame } = incomingFrames[i];
                document.getElementById('vlan-status-'+i).innerText = (''+vlan);
                if (frame)
                    enqueueFrame(frame, i, false);
            }
        }
        catch (e) { frameworkError(e) ; return; }
        await nextTick;
    }
})();

/* render controls */
const sendPacket = ((src, dest, checksumOK = true) =>
{
    hosts[src].sendQueue.push({
        target: hosts[dest].address,
        source: hosts[src].address,
        etherType: 0xffff,
        data: Array.from(new Uint8Array(new Uint32Array([window.globalFrameCounter++]).buffer)),
        checksumOK: checksumOK,
    });
});

const controlTable = document.getElementById('controls-table');

const topDescrRow = makeElm('tr', controlTable);
makeElm('td', topDescrRow, 'noborder').colSpan = 2;
const topDescrCell = makeElm('th', topDescrRow);
topDescrCell.innerText = 'Sent to:'
topDescrCell.colSpan = (N_PORTS+1);

const topLabelRow = makeElm('tr', controlTable);
makeElm('td', topLabelRow, 'noborder').colSpan = 2;
for (let i=0; i<N_PORTS; ++i)
    makeElm('th', topLabelRow).innerText = (''+i).padStart(2,'0');
makeElm('th', topLabelRow).innerText = 'B';

for (let src=0; src<N_PORTS; ++src)
{
    const row = makeElm('tr', controlTable);
    if (src === 0)
    {
        const descr = makeElm('th', row, 'rotate');
        descr.innerText = 'Sent by:';
        descr.rowSpan = N_PORTS;
    }
    makeElm('th', row).innerText = (''+src).padStart(2,'0');
    for (let dest=0; dest<=N_PORTS; ++dest)
    {
        const cell = makeElm('td', row);
        const btn = makeElm('button', cell)
        btn.innerText = 'o';
        btn.addEventListener('click', (e) =>
        {
            sendPacket(src, dest, e.button !== 2);
            e.preventDefault();
        });
        btn.addEventListener('contextmenu', (e) =>
        {
            sendPacket(src, dest, false);
            e.preventDefault();
        });

        if (src === dest)
            btn.disabled = true;
    }
}

const vlanTable = document.getElementById('vlan-table');

const labelRow = makeElm('tr', vlanTable);
const statusRow = makeElm('tr', vlanTable);
const changeRow = makeElm('tr', vlanTable);

makeElm('th', labelRow).innerText = 'Port:';
makeElm('th', statusRow).innerText = 'VLAN:';
makeElm('th', changeRow).innerText = 'Set to:';

for (let i=0; i<N_PORTS; ++i)
{
    makeElm('th', labelRow).innerText = (''+i).padStart(2,'0');
    
    const txt = makeElm('td', statusRow);
    txt.innerText = '…';
    txt.id = ('vlan-status-'+i);
    
    const sel = makeElm('select', makeElm('td', changeRow));
    sel.id = ('vlan-update-'+i);
    for (let j=0; j<N_PORTS; ++j)
    {
        const o = makeElm('option', sel);
        o.value = j;
        o.innerText = j;
    }
    
}

/* data generation */
let generateEnabled = false;
const dataStartButton = document.getElementById('data-start');
const dataStopButton = document.getElementById('data-stop');
(async () =>
{
    let first = 0;
    while (true)
    {
        if (generateEnabled)
        {
            dataStartButton.classList.add('generating');
            dataStartButton.disabled = true;
            dataStartButton.innerText = 'Generating...'
            dataStopButton.disabled = false;
            try
            {
                if (first < N_PORTS)
                {
                    if (!first)
                        sendPacket(first, N_PORTS);
                    else
                        sendPacket(first, Math.floor(Math.random()*first));
                    ++first;
                }
                else
                {
                    const src = Math.floor(Math.random() * N_PORTS);
                    const dst = Math.floor(Math.random() * N_PORTS);
                    sendPacket(src, (src === dst) ? N_PORTS : dst);
                }
                await sleep(5500);
            }
            catch (e) { console.error(e); }
        }
        else
        {
            dataStartButton.classList.remove('generating');
            dataStartButton.disabled = false;
            dataStartButton.innerText = 'Auto-generate data';
            dataStopButton.disabled = true;
        }
        await sleep(100);
    }
})();

dataStartButton.addEventListener('click', () =>
{
    generateEnabled = true;
    dataStartButton.disabled = true;
});

dataStopButton.addEventListener('click', () =>
{
    generateEnabled = false;
    dataStopButton.disabled = true;
});
