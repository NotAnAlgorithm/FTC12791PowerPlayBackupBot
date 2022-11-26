package org.firstinspires.ftc.teamcode.webserver;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import fi.iki.elonen.NanoWSD;

public class TerminalServer extends NanoWSD {
    private static final int PORT = 8000;
    public static final String DEFAULT_FILE = "dash/index.html";

    protected Terminal terminal;
    protected final List<TerminalWebSocket> sockets = new ArrayList<>();

    public TerminalServer(Terminal terminal) {
        super(PORT);
        this.terminal = terminal;
    }

    @Override
    protected WebSocket openWebSocket(IHTTPSession handshake) {
        return new TerminalWebSocket(handshake);
    }


    public class TerminalWebSocket extends WebSocket {
        public static final String TAG = "TerminalWebSocket";

        public TerminalWebSocket(IHTTPSession handshakeRequest) {
            super(handshakeRequest);
        }

        @Override
        protected void onOpen() {
            synchronized (sockets) {
                sockets.add(this);
            }
        }

        @Override
        protected void onClose(WebSocketFrame.CloseCode code, String reason, boolean initiatedByRemote) {
            synchronized (sockets) {
                sockets.remove(this);
            }
        }

        @Override
        protected void onMessage(WebSocketFrame message) {

        }

        @Override
        protected void onPong(WebSocketFrame pong) {

        }

        @Override
        protected void onException(IOException exception) {

        }
    }
}
