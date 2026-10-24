// Browser microphone -> local ABR in WSL. No cloud speech service.
(() => {
  const card = document.createElement('section');
  card.style.cssText = 'margin:20px;padding:20px;border:1px solid #39404b;border-radius:14px;background:#15181d';
  card.innerHTML = `<h3>VOICE · ABR</h3><button id="voice-toggle">Voice OFF</button>
    <button id="voice-stop">STOP</button><span id="voice-status">Microphone off</span>
    <label><input id="gemini-on" type="checkbox" checked> Gemini understanding</label>
    <label><input id="reply-on" type="checkbox" checked> ABR spoken replies</label>
    <p>With Gemini on, your message text is sent to Google. Audio stays on this laptop.</p>
    <p id="runtime-status" role="status">Checking speech services…</p>
    <div id="voice-examples" style="display:flex;flex-wrap:wrap;gap:6px;margin:12px 0"></div>
    <p>Say or type a command. Examples fill the box; press Send to run.</p>
    <style>
      #voice-layout {display:grid;grid-template-columns:minmax(0,1fr) 290px;gap:18px}
      #voice-guide {padding:16px;background:#202630;border-radius:10px}
      #voice-guide li {margin:12px 0;line-height:1.5}
      #voice-guide code {color:#a9e2b0}
      #voice-form {display:flex;gap:8px;margin-top:10px}
      #voice-text {flex:1;min-width:0;padding:12px}
      @media(max-width:760px) {#voice-layout {grid-template-columns:1fr}}
    </style>
    <div id="voice-layout"><div>
      <div id="voice-chat" role="log" aria-live="polite" style="min-height:260px;max-height:360px;overflow:auto;padding:10px;background:#0e1116;border-radius:10px"></div>
      <form id="voice-form"><input id="voice-text" placeholder="Type a command…" maxlength="200" autocomplete="off"><button>Send</button></form>
    </div><aside id="voice-guide" aria-label="Things you can say">
      <h4 style="margin-top:0">THINGS YOU CAN SAY</h4>
      <ul style="padding-left:18px">
        <li><code>Turn left for 3 seconds</code></li>
        <li><code>Turn right for 0.5 seconds</code></li>
        <li><code>Move forward for 2 seconds</code></li>
        <li><code>Move backwards for 1 second</code></li>
        <li><code>Stop</code></li>
        <li><code>Back up a little, then turn left</code></li>
        <li><code>Make your lights purple</code></li>
        <li><code>Flash green</code></li>
        <li><code>Show a blue snake pattern</code></li>
        <li><code>Show an orange chase pattern</code></li>
        <li><code>Make a rainbow light show</code></li>
        <li><code>Make the purple lights breathe</code></li>
        <li><code>Turn right for 1 second, go back for 3 seconds, repeat twice</code></li>
        <li><code>Restore your status lights</code></li>
      </ul>
      <p>Replace the time with <strong>0.5, 1, 2, 3, 4 or 5 seconds</strong>.</p>
      <p>You can say “half a second” or “three seconds” too.</p>
      <p>Always include a time for predictable movement. Without a time, Gemini uses a short 0.5-second movement; basic mode uses 1 second.</p>
      <p>Say one command, then pause. Check the chat shows the words you intended.</p>
      <p>Up to 6 steps per sequence. Repeat 2 or 3 times, with at most 12 steps and 10 seconds of movement overall. Lights return to status colors after 15 seconds.</p>
    </aside></div>`;
  document.body.append(card);
  const $ = id => document.getElementById(id);
  async function runtimeStatus() {
    try {
      const response=await fetch('/api/startup');
      if (!response.ok) throw new Error();
      const data=await response.json();
      $('runtime-status').textContent=`ABR: ${data.abr} · Gemini: ${data.gemini}`;
      if (data.abr==='ready') clearInterval(runtimeTimer);
    } catch (_) { $('runtime-status').textContent='Speech service status unavailable'; }
  }
  const runtimeTimer=setInterval(runtimeStatus,2000);
  runtimeStatus();
  for (const command of ['turn right for 0.5 seconds', 'turn left for 0.5 seconds', 'move forward for 0.5 seconds', 'move backwards for 0.5 seconds', 'show a blue snake pattern', 'show a rainbow pattern', 'stop']) {
    const button = document.createElement('button'); button.type = 'button'; button.textContent = command;
    button.onclick = () => { $('voice-text').value = command; $('voice-text').focus(); };
    $('voice-examples').append(button);
  }
  let on = false, generation = 0, context, stream, processor, source, mute;
  let chunks = [], samples = 0, silence = 0, speaking = false, recognizing = false;
  let action = null;
  let planEpoch = 0, planning = false, talking = false, speaker = null, finishSpeech = null;
  let speechSerial = 0, speechAbort = null;
  function cancelSpeech() {
    speechSerial++;
    speechAbort?.abort(); speechAbort = null;
    if (speaker) { speaker.pause(); speaker = null; }
    if (finishSpeech) finishSpeech();
    talking=false; resetAudio();
  }
  async function spokenReply(text, epoch, plan) {
    if (!$('reply-on').checked) return;
    cancelSpeech();
    const serial=speechSerial, abort=new AbortController(); speechAbort=abort;
    const timeout=setTimeout(()=>abort.abort(),30000);
    try {
      const response = await fetch('/api/voice/speak', {method:'POST', signal:abort.signal, headers:{'Content-Type':'application/json'},body:JSON.stringify({text})});
      if (serial!==speechSerial) return;
      if (!response.ok) { log('ABR','Spoken reply unavailable; see text above.'); return; }
      const blob = await response.blob();
      if (epoch !== generation || plan !== planEpoch || serial!==speechSerial) return;
      talking=true; resetAudio();
      const url = URL.createObjectURL(blob);
      try {
        await new Promise(resolve => {
          const audio = new Audio(url); speaker = audio;
          let timer;
          const finish = () => { clearTimeout(timer); audio.pause(); if(speaker===audio) speaker=null; finishSpeech=null; resolve(); };
          finishSpeech=finish; audio.onended=finish; audio.onerror=finish;
          timer=setTimeout(finish,30000);
          audio.play().catch(() => { log('ABR','Browser blocked playback. Text replies still work.'); finish(); });
        });
      } finally { URL.revokeObjectURL(url); }
    } catch (_) { if(serial===speechSerial && !abort.signal.aborted) log('ABR','Spoken reply connection failed.'); }
    finally { clearTimeout(timeout); if(serial===speechSerial) { speechAbort=null; talking=false; resetAudio(); } }
  }
  function log(who, text) {
    const p = document.createElement('p'); p.textContent = `${who}: ${text}`;
    p.style.cssText = `padding:8px 12px;border-radius:10px;max-width:90%;white-space:pre-wrap;${who==='You' ? 'margin-left:auto;background:#224435' : 'margin-right:auto;background:#252d3b'}`;
    $('voice-chat').append(p);
    while ($('voice-chat').children.length > 60) $('voice-chat').firstChild.remove();
    $('voice-chat').scrollTop = $('voice-chat').scrollHeight;
  }
  function status(text) { $('voice-status').textContent = ' · ' + text; }
  async function halt(disableRobot=false, completed=false) {
    const old = action; action = null;
    if (old) { clearTimeout(old.timer); clearTimeout(old.timerPump); await stopManual(); log('Robot', 'Stop sent.'); old.done?.(completed); }
    if (disableRobot) { await post('/api/disable'); log('Robot', 'Disable requested'); }
  }
  async function execute(text, epoch) {
    log('You', text || '(no speech detected)');
    if (/^(?:(?:robot|please)\s+)*(?:stop|stop moving|disable|emergency stop)[.!?]*$/i.test(text.trim())) {
      planEpoch++; cancelSpeech(); await halt(true); return;
    }
    if (!text.trim()) return;
    if (planning) { log('Robot','Another request is active. Say Stop to cancel.'); return; }
    planning=true; const plan=++planEpoch;
    try {
      const ai=$('gemini-on').checked;
      status(ai ? 'Gemini thinking…' : 'Reading command…');
      const parsed = await post(ai ? '/api/voice/plan' : '/api/voice/parse', {text});
      if (epoch!==generation || plan!==planEpoch) return;
      if (!parsed?.success) { log('Robot',parsed?.message || 'No plan returned'); return; }
      const actions=ai ? parsed.actions : [{kind:parsed.command.direction==='stop'?'stop':'move',...parsed.command}];
      if (actions.some(a=>a.kind!=='stop') && !enabled) { log('Robot','Blocked: Enable the robot first.'); return; }
      if (actions.length && (manualToken || drawMode || zeroPicking)) { log('Robot','Finish manual driving, drawing or heading setup first.'); return; }
      const reply=ai ? parsed.reply : `Requested ${parsed.command.direction} for ${parsed.command.seconds} seconds.`;
      log(ai?'Gemini':'Robot',reply);
      if (actions.length===1 && actions[0].kind==='stop') { planEpoch++; cancelSpeech(); await halt(true); return; }
      // Speech synthesis and playback must never delay motor commands.
      void spokenReply(reply,epoch,plan);
      for (const [index, step] of actions.entries()) {
        if (epoch!==generation || plan!==planEpoch || document.hidden) return;
        if (!enabled) { log('Robot','Stopped: robot is disabled.'); return; }
        if (step.kind==='move') { if (!await moveOne(step,epoch)) return; }
        else if (step.kind==='lights') {
          const result=await post('/api/lights',{color:step.color,effect:step.effect});
          if (!result?.success) { log('Robot',result?.message || 'Light command failed'); return; }
          log('Robot',`Light command sent: ${step.effect} ${step.color}`);
          // Give adjacent light effects time to be seen before replacing one.
          await new Promise(resolve=>setTimeout(resolve,actions[index+1]?.kind==='lights' ? 2000 : 150));
        }
      }
      if (actions.length) log('Robot','Plan finished; stop commands sent after movement.');
    } finally { planning=false; if(on) status('Listening'); else status('Microphone off'); }
  }
  async function moveOne({direction, seconds}, epoch) {
    if (!enabled) { log('Robot', 'Blocked: Enable the robot first.'); return false; }
    if (manualToken || action) { log('Robot', 'Blocked: another movement is active.'); return; }
    if (drawMode || zeroPicking) { log('Robot', 'Finish drawing or heading setup first.'); return; }
    const token = crypto.randomUUID();
    const run = {token, deadline: performance.now() + seconds * 1000};
    const finished=new Promise(resolve=>run.done=resolve);
    action = run; manualToken = token;
    log('Robot', `Requesting ${direction} for ${seconds} seconds`);
    run.timer = setTimeout(() => { if (action === run) halt(false,true); }, seconds * 1000);
    async function heartbeat() {
      if (action !== run || manualToken !== token || epoch !== generation || document.hidden) {
        if (action === run) await halt(); return;
      }
      if (performance.now() >= run.deadline) { await halt(false,true); return; }
      const result = await post('/api/manual', {token, direction, sent_ms:Date.now()});
      if (action !== run) return;
      if (!result?.success) { log('Robot', 'Movement blocked: ' + (result?.message || 'connection lost')); await halt(); return; }
      if (!run.accepted) {
        run.accepted = true;
        log('Robot', `${direction==='left'||direction==='right' ? 'Turning' : 'Moving'} ${direction} for ${seconds} seconds — command accepted.`);
      }
      run.timerPump = setTimeout(heartbeat, 60);
    }
    await heartbeat();
    return finished;
  }
  function wav(parts, rate) {
    const input = new Float32Array(parts.reduce((n, p) => n + p.length, 0));
    let offset=0; for (const p of parts) { input.set(p, offset); offset+=p.length; }
    const count=Math.floor(input.length*16000/rate), buffer=new ArrayBuffer(44+count*2), view=new DataView(buffer);
    function str(at, value) { for(let i=0;i<value.length;i++) view.setUint8(at+i,value.charCodeAt(i)); }
    str(0,'RIFF'); view.setUint32(4,36+count*2,true); str(8,'WAVE'); str(12,'fmt ');
    view.setUint32(16,16,true); view.setUint16(20,1,true); view.setUint16(22,1,true);
    view.setUint32(24,16000,true); view.setUint32(28,32000,true); view.setUint16(32,2,true); view.setUint16(34,16,true);
    str(36,'data'); view.setUint32(40,count*2,true);
    for(let i=0;i<count;i++) { const x=i*rate/16000, a=Math.floor(x), f=x-a;
      const v=input[a]*(1-f)+(input[Math.min(a+1,input.length-1)]||0)*f;
      view.setInt16(44+i*2,Math.max(-1,Math.min(1,v))*32767,true); }
    return buffer;
  }
  async function recognize(parts, rate, epoch) {
    recognizing=true; status('ABR recognizing…');
    try {
      const response=await fetch('/api/voice/transcribe', {method:'POST',headers:{'Content-Type':'audio/wav'},body:wav(parts,rate)});
      const data=await response.json();
      if (!on || epoch!==generation) return;
      if (!response.ok) log('ABR',data.message || 'Recognition failed');
      else { recognizing=false; await execute(data.text, epoch); }
    } catch (_) { if (epoch===generation) log('ABR','Connection failed'); }
    finally { recognizing=false; if(on) status('Listening'); }
  }
  function resetAudio() { chunks=[]; samples=0; silence=0; speaking=false; }
  async function off() {
    on=false; generation++; planEpoch++; cancelSpeech(); resetAudio(); await halt();
    processor?.disconnect(); source?.disconnect(); mute?.disconnect();
    stream?.getTracks().forEach(track=>track.stop());
    if(context && context.state!=='closed') await context.close();
    $('voice-toggle').textContent='Voice OFF'; status('Microphone off');
  }
  $('voice-toggle').onclick=async()=> {
    if(on) { await off(); return; }
    try {
      stream=await navigator.mediaDevices.getUserMedia({audio:{channelCount:1,echoCancellation:true,noiseSuppression:true}});
      context=new AudioContext(); await context.resume(); source=context.createMediaStreamSource(stream);
      processor=context.createScriptProcessor(4096,1,1); mute=context.createGain(); mute.gain.value=0;
      source.connect(processor); processor.connect(mute); mute.connect(context.destination);
      on=true; const epoch=++generation; $('voice-toggle').textContent='Voice ON'; status('Listening');
      log('Microphone', 'Listening. Say a command, then pause. Your words and the action will appear here.');
      processor.onaudioprocess=e=> {
        if(!on || recognizing || talking || epoch!==generation) { resetAudio(); return; }
        const data=e.inputBuffer.getChannelData(0); let power=0; for(const v of data) power+=v*v;
        const loud=Math.sqrt(power/data.length)>.012;
        if(loud) { speaking=true; silence=0; }
        if(!speaking) return;
        chunks.push(new Float32Array(data)); samples+=data.length;
        if(!loud) silence+=data.length;
        if(silence>context.sampleRate*.65 || samples>context.sampleRate*6) {
          const parts=chunks; resetAudio(); recognize(parts,context.sampleRate,epoch);
        }
      };
    } catch(e) { await off(); log('Microphone', e.name==='NotAllowedError' ? 'Allow microphone access in the browser, then press Voice ON again. Typed commands still work.' : e.message); }
  };
  $('voice-stop').onclick=()=> { generation++; resetAudio(); off(); post('/api/disable'); log('Robot','Disable requested'); };
  $('voice-form').onsubmit=async e=> { e.preventDefault(); const text=$('voice-text').value.trim(); $('voice-text').value=''; if(text) await execute(text,generation); };
  window.addEventListener('voice-cancel',()=> { planEpoch++; cancelSpeech(); halt(); });
  document.addEventListener('click',event=> {
    if (event.target.closest('button') && !card.contains(event.target)) {
      planEpoch++; cancelSpeech(); halt();
    }
  },true);
  $('gemini-on').onchange=()=> { planEpoch++; cancelSpeech(); halt(); };
  window.addEventListener('blur',()=> { if(on || action) off(); });
  window.addEventListener('pagehide',()=> { generation++; on=false; if(action) halt(); stream?.getTracks().forEach(t=>t.stop()); });
  document.addEventListener('visibilitychange',()=> { if(document.hidden && (on || action)) off(); });
})();
