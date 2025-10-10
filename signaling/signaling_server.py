# Minimal relay: one "browser" <-> one "robot"
import asyncio, json, websockets

BROWSER = None
ROBOT = None

async def handler(ws):
    global BROWSER, ROBOT
    role = None
    try:
        # Expect the first message to set the role
        hello = json.loads(await ws.recv())
        role = hello.get("role")
        if role == "browser":
            BROWSER = ws
        elif role == "robot":
            ROBOT = ws
        else:
            await ws.close()
            return

        async for raw in ws:
            msg = json.loads(raw)
            # forward to the other side
            if role == "browser" and ROBOT:
                await ROBOT.send(raw)
            elif role == "robot" and BROWSER:
                await BROWSER.send(raw)
    except:
        pass
    finally:
        if role == "browser" and BROWSER == ws: BROWSER = None
        if role == "robot" and ROBOT == ws: ROBOT = None

async def main():
    async with websockets.serve(handler, "", 8765):
        print("Signaling on ws://0.0.0.0:8765")
        await asyncio.Future()

asyncio.run(main())
