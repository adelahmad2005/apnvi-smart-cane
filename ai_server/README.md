
# AI server and device setup guide, step by step

Follow the steps in order. Each step tells you what to **do**, gives you a grey box to
**copy and paste**, and tells you what you **should see**. If you do not see it, read
**If not** and stop there.

**How to copy a grey box:** select the text inside it with the mouse, press **Ctrl + C**,
click where you want it, press **Ctrl + V**, then press **Enter**. Paste one box at a time.

| Part | Where | About how long |
|---|---|---|
| A | The campus PC (Windows) | 1 hour, mostly waiting for downloads |
| B | The Jetson | 30 minutes |
| C | Optional: make speech faster on the campus PC | 20 minutes |
| D | The campus PC: start the speech program by itself at login | 5 minutes |
| E | The campus PC and the Jetson: the natural voice (Kokoro) | 20 minutes |

**What you need before starting:** the campus PC with internet, an administrator password
for it, the Jetson with its screen and keyboard, a folder of test photos, the microphone.

---

## Part A — the campus PC

### A1. Open the Command Prompt

**Do:** click the Start button (bottom left), type `cmd`, and press Enter.

**You should see:** a black window with text like `C:\Users\...>` and a blinking cursor.
This is the **Command Prompt**. Most grey boxes below get pasted here.

### A2. Write down the PC's address

**Do:** paste this into the Command Prompt.

```
ipconfig
```

**You should see:** several lines. Find the line **IPv4 Address** and write down the number
after it, for example `10.20.30.40`. This guide calls it **PC_IP**. You need it in Part B.

**If not:** if there are several IPv4 lines, use the one under the section that says
Ethernet or Wi-Fi and is not "Media disconnected".

### A3. Install Ollama (the program that runs the AI)

**Do:** open a web browser, go to **ollama.com/download**, click **Download for Windows**,
open the downloaded file and click **Install**. Accept every question.

**You should see:** a small llama icon near the clock (bottom right). You may need to click
the little arrow ^ to see it.

### A4. Download the AI model

**Do:** go back to the Command Prompt and paste:

```
ollama pull qwen3-vl:4b-instruct
```

**You should see:** a progress bar. It downloads about 3 GB. At the end it says
**success**.

**If not:** if it says `'ollama' is not recognized`, close the Command Prompt, open a new
one (step A1), and try again.

### A5. Let the Jetson talk to Ollama

Ollama only answers this PC unless we tell it otherwise.

**Do:**
1. Right-click the llama icon near the clock and click **Quit Ollama**.
2. Click Start, type `environment variables`, and click **Edit environment variables for
   your account**.
3. In the top half, click **New...**
4. Variable name: `OLLAMA_HOST`   Variable value: `0.0.0.0`   Click **OK**, then **OK**.
5. Click Start, type `Ollama`, and open it again.

**You should see:** the llama icon is back near the clock.

### A6. Check Ollama works

**Do:** in the web browser, go to this address:

```
http://localhost:11434
```

**You should see:** a page that only says **Ollama is running**.

**If not:** wait 30 seconds and refresh. If it still fails, repeat A5.

### A7. Install Python

**Do:** in the browser go to **python.org/downloads/windows**, click the newest
**Python 3.11** release, and download **Windows installer (64-bit)**. Open it.
**Important: on the first screen tick the box "Add python.exe to PATH"**, then click
**Install Now**.

**You should see:** "Setup was successful". Click Close.

### A8. Download our code

**Do:**
1. In the browser go to **github.com/adelahmad2005/apnvi-smart-cane**.
2. Click the green **Code** button, then **Download ZIP**.
3. Make a folder called `apnvi` directly on the C: drive (open **This PC**, **Local Disk
   (C:)**, right-click, **New**, **Folder**).
4. Right-click the downloaded ZIP, click **Extract All...**, type `C:\apnvi`, click
   **Extract**.

**You should see:** a folder `C:\apnvi\apnvi-smart-cane-main` with folders like `src`
and `ai_server` inside. The code for this guide is in
`C:\apnvi\apnvi-smart-cane-main\ai_server`.


### A9. Install what the speech program needs

**Do:** close the Command Prompt, open a new one (step A1), and paste these two lines,
one at a time:

```
cd /d C:\apnvi\apnvi-smart-cane-main\ai_server
```

```
python -m pip install -r requirements.txt
```

**You should see:** many lines scroll past, ending with **Successfully installed ...**

**If not:** if it says `'python' is not recognized`, Python was installed without the
"Add python.exe to PATH" tick. Run the Python installer again, choose **Modify**, and make
sure that option is on.

### A10. Start the speech program

**Do:** in the same Command Prompt, paste:

```
python speech_server.py --device cpu
```

**You should see:** "Loading Whisper..." and then, after it downloads its model the first
time, **Ready. Listening on port 8090**.

**Leave this window open.** Closing it stops the speech program. Open a new Command Prompt
for the next steps.

*(This uses the processor. Part C makes it faster with the graphics card, later.)*

### A11. Check the speech program works

**Do:** in the browser go to:

```
http://localhost:8090/health
```

**You should see:** a line containing `"ok": true`.

### A12. Open the firewall for the Jetson

**Do:** click Start, type `cmd`, **right-click** Command Prompt and choose **Run as
administrator**. Enter the administrator password. Then paste these two lines, one at a
time:

```
netsh advfirewall firewall add rule name="APNVI Ollama" dir=in action=allow protocol=TCP localport=11434
```

```
netsh advfirewall firewall add rule name="APNVI speech" dir=in action=allow protocol=TCP localport=8090
```

**You should see:** **Ok.** after each line.

**If not:** "The requested operation requires elevation" means this window is not running
as administrator. Close it and open it again with right-click, Run as administrator.

### A13. Try the AI on the test photos

**Do:**
1. Copy Mo'ath's photos into a new folder `C:\apnvi\photos`.
2. Open a new Command Prompt (step A1) and paste, one at a time:

```
python -m pip install numpy opencv-python
```

```
cd /d C:\apnvi\apnvi-smart-cane-main
```

```
python tools\stage2\try_finder.py C:\apnvi\photos
```

**You should see:** one line per photo, with `found=yes` or `found=no` and the seconds it
took. The first photo is slow because the AI is loading.

**Then:** open the folder `C:\apnvi\photos\results`. Each photo has a red box drawn where
the AI thinks the thing is. Count how many boxes are on the right thing and write it down.

**If answers take more than about 3 seconds each:** try the smaller AI. Paste these, one
at a time:

```
ollama pull qwen3-vl:2b
```

```
python tools\try_finder.py C:\apnvi\photos --model qwen3-vl:2b
```

**Part A is done.** Keep the speech window from A10 open.

---

## Part B — the Jetson

### B1. Open a Terminal

**Do:** on the Jetson press **Ctrl + Alt + T**.

**You should see:** a window with text ending in `$`. This is the **Terminal**. Paste here
with **Ctrl + Shift + V** (not Ctrl + V).

Make sure the Jetson is on the campus wifi (wifi icon, top right).

### B2. Check the Jetson can reach the PC

**Do:** paste these two lines, one at a time. **Replace PC_IP with the number from A2**
before pressing Enter.

```
curl http://PC_IP:11434
```

```
curl http://PC_IP:8090/health
```

**You should see:** the first prints **Ollama is running**. The second prints a line with
`"ok": true`.

**If not:** if it waits for a long time or says "refused": check A5 and A12 on the PC, and
that the speech window from A10 is still open.

### B3. Get the new code and build it

**Do:** paste these lines, one at a time. Enter the Jetson's password if asked (nothing
shows while you type it, that is normal).

```
git clone https://github.com/adelahmad2005/apnvi-smart-cane.git ~/apnvi_demo/src/apnvi-smart-cane
```

```
bash ~/apnvi_demo/src/apnvi-smart-cane/jetson_setup/jetson_setup.sh
```

This moves the old code folder out of the way (it is kept in the home folder as
`old-apnvi-stage1-...`), installs what is missing, and builds Stage 1 and Stage 2.

**You should see:** **Summary: 2 packages finished**, then **SETUP FINISHED - ALL GOOD**.
Then type `source ~/.bashrc` once, so the `apnvi` shortcut points to the new code.

**Later updates:** paste only the last line (`bash ~/apnvi_demo/...jetson_setup.sh`).

**If not:** if the second line says `already exists`, skip it and paste the last line.

### B4. Put the new program on the ESP32

**Do:**
1. On the laptop with the Arduino app, download the code: go to
   **github.com/adelahmad2005/apnvi-smart-cane**, log in, click **Code**, then **Download ZIP**,
   and extract it. Open the file `firmware\apnvi_esp32\apnvi_esp32.ino`
   in the Arduino app.
2. Click **Verify** (the tick). Wait for **Done compiling**.
3. Plug in the ESP32, click **Upload** (the arrow). Wait for **Done uploading**.
4. Open **Tools**, **Serial Monitor**, and set the speed to **115200**.

**You should see:** numbers like `1.23` scrolling, as before. Press the button: a line
**B1** appears. Let go: **B0** appears.

5. Close the Serial Monitor and plug the ESP32 back into the Jetson.

### B5. Find the microphone's name

**Do:** plug the microphone into the Jetson and paste:

```
arecord -l
```

**You should see:** a list of cards. Find the microphone's line, for example
`card 2: Device [USB Audio Device], device 0`. Its name is **plughw:** then the card
number, a comma, and the device number. In this example: `plughw:2,0`. This guide calls it
**MIC**.

### B6. Test the microphone

**Do:** paste this (replace MIC), and speak for 3 seconds:

```
arecord -D MIC -f S16_LE -r 16000 -c 1 -d 3 test.wav && aplay test.wav
```

**You should see:** after 3 seconds, you hear your own voice played back.

### B7. Start everything

**Do:** paste this, replacing **PC_IP** and **MIC**:

```
ros2 launch apnvi_stage2 stage2.launch.py pc:=PC_IP mic:=MIC
```

**You should see:** many lines scroll, including **Ready. Qwen at http://...**
**Leave this window open.**

(If you used the smaller AI in A13, add ` model:=qwen3-vl:2b` at the end.)

### B8. Open the screens

**Do:** on a laptop on the same wifi, open the browser. You need the Jetson's address:
open a new Terminal on the Jetson (Ctrl + Alt + T) and paste `hostname -I`. Use the first
number. Then go to:

```
http://JETSON_IP:8081
```

**You should see:** the live colour camera and a green **READY** box underneath.
(Stage 1's screen is still at port 8080.)

### B9. Ask by typing (first test)

**Do:** in the second Jetson Terminal, paste these, one at a time:

```
source ~/apnvi_demo/install/setup.bash
```

```
ros2 topic pub --once /voice/question std_msgs/String "{data: 'where is the exit'}"
```

**You should see:** on the screen: THINKING, then the picture with a red box around the
exit, or the device says **"I can't see it, turn slowly and try again"**.

### B10. Ask with the button (real test)

**Do:** point the camera at something on the list, **hold** the button, say "where is the
exit", **let go**.

**You should see:** LISTENING while you hold, THINKING after you let go, then the red box
(silent) or the "turn slowly" sentence.

### B11. Safety check

**Do:** turn the Jetson's wifi off. Walk the device toward a chair.

**You should see:** Stage 1 still beeps faster as it gets closer. Ask a question with the
button: the device says **"I can't reach the helper computer"**. Turn the wifi back on.

**All done.** Write down for each target: did the box land on it, and how many seconds it
took (shown on the screen).

---

## Part C — optional: faster speech using the graphics card

Only do this after everything above works. It makes speech about 5 times faster (tested 12 Oct:
about 0.1 s instead of 0.7 s). It downloads about 1 GB.

### C1. Download NVIDIA's two libraries

**Do:** open a new Command Prompt (step A1) and paste:

```
python -m pip download nvidia-cublas-cu12==12.4.5.8 nvidia-cudnn-cu12==9.5.1.17 --no-deps --only-binary=:all: --platform win_amd64 -d C:\apnvi\downloads\nvidia
```

**You should see:** two downloads, ending with **Successfully downloaded**. Two files with
names ending in `.whl` are now in `C:\apnvi\downloads\nvidia`.

### C2. Unpack them

**Do:** paste these, one at a time:

```
mkdir C:\apnvi\downloads\nvidia\unpacked
```

```
tar -xf C:\apnvi\downloads\nvidia\nvidia_cublas_cu12-12.4.5.8-py3-none-win_amd64.whl -C C:\apnvi\downloads\nvidia\unpacked
```

```
tar -xf C:\apnvi\downloads\nvidia\nvidia_cudnn_cu12-9.5.1.17-py3-none-win_amd64.whl -C C:\apnvi\downloads\nvidia\unpacked
```

**You should see:** no message. That is normal.

**If not:** if `tar` says a file does not exist, open the folder `C:\apnvi\downloads\nvidia`
and check the exact file names. On 12 Oct these files were unpacked a different way, so this
`tar` step has not been tried yet.

### C3. Copy the library files into one folder

**Do:** paste these, one at a time:

```
mkdir C:\apnvi\cuda_libs
```

```
copy C:\apnvi\downloads\nvidia\unpacked\nvidia\cublas\bin\*.dll C:\apnvi\cuda_libs
```

```
copy C:\apnvi\downloads\nvidia\unpacked\nvidia\cudnn\bin\*.dll C:\apnvi\cuda_libs
```

**You should see:** "3 file(s) copied" after the first copy and "8 file(s) copied" after the
second.

### C4. Tell Windows where the folder is

**Do:** click Start, type `environment variables`, open **Edit environment variables for your
account**, click **Path** in the top half, **Edit...**, **New**, type `C:\apnvi\cuda_libs`,
**OK**, **OK**. (Do not use the `setx` command for this: it can cut the Path short.)

### C5. Restart the speech program on the graphics card

**Do:** close the speech window from A10 and **every** Command Prompt. Open a new one and paste,
one at a time:

```
cd /d C:\apnvi\apnvi-smart-cane-main\ai_server
```

```
python speech_server.py
```

**You should see:** `Loading Whisper "base" on cuda`, then **Ready. Listening on port 8090**.

**If not:** any error mentioning cuBLAS, cuDNN or CUDA means it did not work. Go back to
`python speech_server.py --device cpu`, which works fine, only slower.

---

## Part D — start the speech program by itself at login

Ollama already starts by itself. Without this part, after the PC restarts someone has to start
the speech program by hand, and until then the device says "I can't reach the helper computer".

### D1. Make the start file

**Do:**
1. Press **Windows key + R**, type `shell:startup`, press Enter. A folder opens.
2. In that folder right-click, **New**, **Text Document**. Name it `apnvi_speech.bat`. Click
   **Yes** if Windows asks about changing the extension.
3. Right-click it, **Edit**, paste these two lines, save and close:

```
cd /d C:\apnvi\apnvi-smart-cane-main\ai_server
start "APNVI speech" cmd /k python speech_server.py
```

If you skipped Part C, add ` --device cpu` at the end of the second line.

### D2. Test it

**Do:** close the speech window that is open now. Then double-click `apnvi_speech.bat`.

**You should see:** a window called "APNVI speech" that says **Ready. Listening on port 8090**.
Only ever have one speech window open.

**Do not move or rename the code folder**, or the start file stops working.

---

## Part E — the natural voice (Kokoro), made on the campus PC

Added 31 Oct 2026 (Adel: a voice that sounds human, made on the PC, in place of espeak-ng).
Set up and tested on the campus PC on 31 Oct, on the graphics card (Adel's pick: about 4 times
faster than the processor). The **voice server** (`voice_server.py`) gets a sentence from
the Jetson and sends back a sound file in a human voice. It runs next to the speech program,
on port **8091**, and changes nothing for the speech program or Ollama. If the PC does not
answer, the Jetson says the sentence with espeak-ng as before.

### E1. Make a separate Python environment for the voice

A **Python environment** is a folder with its own copy of Python and its own add-ons, so the
voice add-ons cannot disturb the speech program's. The graphics card version needs Part C done
first (the `C:\apnvi\cuda_libs` folder).

**Do:** open a new Command Prompt (step A1) and paste, one at a time:

```
python -m venv C:\apnvi\voice_env_gpu
```

```
cd /d C:\apnvi\apnvi-smart-cane\ai_server
```

```
C:\apnvi\voice_env_gpu\Scripts\python.exe -m pip install -r voice_requirements_gpu.txt
```

```
C:\apnvi\voice_env_gpu\Scripts\python.exe -m pip uninstall -y onnxruntime onnxruntime-gpu
```

```
C:\apnvi\voice_env_gpu\Scripts\python.exe -m pip install --no-deps onnxruntime-gpu==1.23.2
```

**You should see:** each install ends with **Successfully installed ...**

The last two lines matter: Kokoro brings along the processor-only onnxruntime, which shares a
folder with the graphics card one. Use 1.23.2, not newer: newer versions need CUDA 13, and this
PC's graphics driver only goes up to CUDA 12.7.

**If not:** if the `cd` line says the path does not exist, use the folder from the first line
of `apnvi_speech.bat` (Part D) instead, with `\campus_pc` at the end.

**Processor only** (no graphics card, or Part C not done): make `C:\apnvi\voice_env` and
install `voice_requirements.txt` instead (one install line, no uninstall). Answers then take
1.2 to 2.1 s instead of 0.3 to 0.5 s; in E2 and E4 use `voice_env` and leave out the `set`
lines and `--device cuda`.

### E2. Start the voice server and listen to it

**Do:** paste, one at a time:

```
set NV=C:\apnvi\voice_env_gpu\Lib\site-packages\nvidia
```

```
set PATH=%NV%\cuda_runtime\bin;%NV%\cufft\bin;%NV%\nvjitlink\bin;C:\apnvi\cuda_libs;%PATH%
```

```
C:\apnvi\voice_env_gpu\Scripts\python.exe voice_server.py --device cuda
```

**You should see:** the first time, "Downloading kokoro-v1.0.onnx" and "Downloading
voices-v1.0.bin" (about 350 MB, into `C:\apnvi\kokoro`). Then some yellow warning lines (normal),
**Runs on CUDAExecutionProvider**, how long a first test sentence took, and **Ready. Listening
on port 8091**. Leave the window open.

**If it says Runs on CPUExecutionProvider:** the graphics card did not start (a red line above
says which file is missing). It still works, only slower. Check the two `set` lines.

**Then:** open a second Command Prompt and paste:

```
curl -o C:\apnvi\hello.wav -H "Content-Type: application/json" -d "{\"text\": \"There's a chair just ahead of you, about 2 metres away.\"}" http://localhost:8091/speak
```

Double-click `C:\apnvi\hello.wav`. It should say the sentence in a natural voice.

Other voices: add `--voice bf_emma` (or any name listed at `http://localhost:8091/health`) to
the start line. `a` = American, `b` = British, `f` = woman, `m` = man. `--speed 1.1` talks a
little faster.

### E3. Open the firewall for the voice

**Do:** open a Command Prompt **as administrator** (step A12) and paste:

```
netsh advfirewall firewall add rule name="APNVI voice" dir=in action=allow protocol=TCP localport=8091
```

**You should see:** **Ok.**

### E4. Start it by itself at login

On the campus PC this is done the same way as the speech program: the file
`C:\APNVI\apnvi_voice.bat`, and a shortcut to it called "APNVI voice" in the Startup folder.

**Do:** make a new text file `C:\apnvi\apnvi_voice.bat` (as in D1) with these four lines:

```
cd /d C:\apnvi\apnvi-smart-cane\ai_server
set NV=C:\apnvi\voice_env_gpu\Lib\site-packages\nvidia
set PATH=%NV%\cuda_runtime\bin;%NV%\cufft\bin;%NV%\nvjitlink\bin;C:\apnvi\cuda_libs;%PATH%
start "APNVI voice" cmd /k C:\apnvi\voice_env_gpu\Scripts\python.exe voice_server.py --device cuda
```

**Then:** press **Windows key + R**, type `shell:startup`, press Enter. Right-click
`apnvi_voice.bat` in `C:\apnvi`, drag it into the Startup folder, and choose **Create shortcuts
here**. Rename the shortcut to "APNVI voice".

**Check:** close the voice window from E2 and double-click the shortcut. A window called
"APNVI voice" says **Runs on CUDAExecutionProvider** and **Ready. Listening on port 8091**. Only
ever have one voice window open.

**Testing on the PC itself:** use `http://127.0.0.1:8091`, not `localhost`, in Python scripts.
On this PC, `localhost` adds about 2 s to every request (Windows tries IPv6 first). `curl` and
the Jetson are not affected.

### E5. Use it on the Jetson

The Jetson needs the code from 31 Oct (`voice.py` with the `voice_engine` setting). Since
31 Oct the Jetson's copy is on `main`, which has it (it was on the branch `stage3-31oct`, which
does not). Then add `voice_engine:=kokoro` to the start line:

```
ros2 launch apnvi_stage2 stage2.launch.py pc:=PC_IP voice_engine:=kokoro
```

**You should hear:** "Listening", "Heard" and the answers in the natural voice. The first line in
the terminal says **Voice: Kokoro from http://PC_IP:8091, espeak-ng if it fails**.

**Check from the Jetson:** `curl http://PC_IP:8091/health` shows `"ok": true`.

Without `voice_engine:=kokoro`, or when the PC does not answer within 3 s, the Jetson speaks with
espeak-ng exactly as before. After the PC fails once, the Jetson uses espeak-ng only for 30 s,
then tries the PC again.

## Part F — the remote link: the Jetson reaches the PC over the internet

**Tested 18 Nov with both machines on AUS wireless** (voice, a guided trip, ssh, the console).
With `pc:=` an
IP address nothing here is used, and the hotspot way (Parts A to E) works as before.

Everyday start on the PC: `start_remote_link.bat` (front door plus ngrok on the fixed address in
`remote_link_domain.txt`). On the Jetson: `pc:=https://<ngrok address>/k/<token>` in
`~/apnvi_device.conf`, and the reporter `console/jetson/remote_link.sh` (stage4) as a boot service,
so `ssh jetson-remote` and the console reach it off the hotspot. The steps F1 to F3 below are the
first test with a Cloudflare quick tunnel; ngrok replaced it for the front door.

The campus firewall blocks connections coming in, so the PC dials out instead. `front_door.py`
puts Qwen, speech and voice behind one address with a token; a Cloudflare quick tunnel gives that
address to the internet. The Jetson's `pc:=` setting takes the whole address.

### F1. Start the front door

```
cd C:\APNVI\apnvi-smart-cane\ai_server
python front_door.py
```

The voice going to the Jetson is squeezed to Opus (about 11 times smaller) with ffmpeg
(`C:\ffmpeg\bin`), and the Jetson unpacks it with GStreamer. Without that a sentence did not
arrive inside the 3 s the Jetson waits, and espeak-ng spoke instead. The user's recording is never
squeezed, so speech to text and Qwen get exactly what they get today.

With a front door as `pc:=`, the Jetson asks a whole question in one trip (`/ask`): the front door
runs the Jetson's own question code (`apnvi_stage2/remote_ask.py`, needs numpy and OpenCV, both on
the campus PC) next to the servers and sends back the answer with its voice. Without numpy or
OpenCV, `/ask` says "not found" and the Jetson asks in four trips by itself.

It prints the token (kept in `front_door_token.txt`, never committed). It listens on this PC only
(127.0.0.1:8095), so it needs no firewall opening. `python front_door.py --new-token` makes a new
token; the old address stops working.

### F2. Start the tunnel (in a second window)

Needs `cloudflared.exe` (free, no account), an install: ask the coordinator or supervisor first
(IT, 12 Oct).

```
C:\APNVI\tools\cloudflared\cloudflared.exe tunnel --no-autoupdate --protocol http2 --url http://127.0.0.1:8095
```

`--protocol http2` is needed: the network blocks the tunnel's normal way out (UDP port 7844), and
without it the tunnel keeps retrying that and never connects (error 1033). TCP port 7844 is open
(test, 18 Nov).

It prints an address like `https://some-words.trycloudflare.com`, then "Registered tunnel
connection ... protocol=http2" when it is ready. The address changes every time the tunnel starts.

### F3. Check from the Jetson, then use it

```
curl https://some-words.trycloudflare.com/k/TOKEN/health
```

**You should see:** `{"front_door": true, "ai": true, "speech": true, "voice": true}`. Then put the
whole address as `pc:=` in `~/apnvi_device.conf`:

```
pc:=https://some-words.trycloudflare.com/k/TOKEN
```

A wrong token gets "not found". The token shows in the Jetson's own log lines: don't paste those
into files that get pushed, or make a new token after.
