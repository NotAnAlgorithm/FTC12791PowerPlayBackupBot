package org.firstinspires.ftc.teamcode.webserver;

import android.app.Activity;
import android.content.Context;
import android.content.res.AssetManager;
import android.util.Log;

import com.qualcomm.robotcore.util.WebHandlerManager;

import org.firstinspires.ftc.ftccommon.external.OnCreate;
import org.firstinspires.ftc.ftccommon.external.OnDestroy;
import org.firstinspires.ftc.ftccommon.external.WebHandlerRegistrar;
import org.firstinspires.ftc.ftccommon.internal.FtcRobotControllerWatchdogService;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.robotcore.internal.webserver.WebHandler;
import org.firstinspires.ftc.robotserver.internal.webserver.MimeTypesUtil;

import java.io.IOException;

import fi.iki.elonen.NanoHTTPD;

public class Terminal {
    public static final String TAG = "Terminal";
    private static Terminal instance;

    public static final String ROOT_URI = "/terminal";

    /**
     * Starts the terminal.
     */
    @OnCreate
    public static void start(Context context) {
        if (instance == null) {
            //todo uncomment this when this is ready to run
//            instance = new Terminal();
        }
    }

    /**
     * Stops the instance and the underlying WebSocket server.
     */
    @OnDestroy
    public static void stop(Context context) {
        if (!FtcRobotControllerWatchdogService.isLaunchActivity(AppUtil.getInstance().getRootActivity())) {
            // prevent premature stop when the app is launched via hardware attachment
            return;
        }

        if (instance != null) {
            instance.close();
            instance = null;
        }
    }

    /**
     * Attaches a web server for accessing the dashboard through the phone (like OBJ/Blocks).
     */
    @WebHandlerRegistrar
    public static void attachWebServer(Context context, WebHandlerManager manager) {
        if (instance == null || manager.getWebServer() == null) return;

        instance.internalAttachWebServer(manager);
    }


    private TerminalServer server;

    private Terminal() {
        server = new TerminalServer(this);
    }

    public void close() {
        server.stop();
    }

    /**
     * Creates a {@link WebHandler} that returns the requested asset if given an HTTP GET method.
     * Returns a blank 404 if a different method is given.
     * @param assetManager Android {@link AssetManager}.
     * @param file String file uri.
     * @return The {@link WebHandler} that returns static assets.
     */
    private WebHandler newStaticAssetHandler(final AssetManager assetManager, final String file) {
        return new WebHandler() {
            @Override
            public NanoHTTPD.Response getResponse(NanoHTTPD.IHTTPSession session) throws IOException, NanoHTTPD.ResponseException {
                switch (session.getMethod()) {
                    case GET:
                        String mimeType = MimeTypesUtil.determineMimeType(file);
                        return NanoHTTPD.newChunkedResponse(NanoHTTPD.Response.Status.OK, mimeType, assetManager.open(file));
                    default:
                        return NanoHTTPD.newFixedLengthResponse(NanoHTTPD.Response.Status.BAD_REQUEST, NanoHTTPD.MIME_PLAINTEXT, "");
                }
            }
        };
    }

    /**
     * In the given directory, create a static asset {@link WebHandler} for each file in the
     * directory (and recursive subdirectories).
     * @param webHandlerManager The {@link WebHandlerManager} to add these WebHandlers to.
     * @param assetManager Android {@link AssetManager}
     * @param path String path uri.
     * @see #newStaticAssetHandler(AssetManager, String)
     */
    private void addAssetWebHandlers(WebHandlerManager webHandlerManager, AssetManager assetManager, String path) {
        try {
            String[] list = assetManager.list(path);

            if (list == null) return;

            if (list.length > 0) {
                for (String file : list) {
                    addAssetWebHandlers(webHandlerManager, assetManager, path + "/" + file);
                }
            } else {
                webHandlerManager.register("/" + path, newStaticAssetHandler(assetManager, path));
            }
        } catch (IOException e) {
            Log.w(TAG, e);
        }
    }

    private void internalAttachWebServer(WebHandlerManager manager) {
        Activity activity = AppUtil.getInstance().getActivity();
        if (activity == null) return;

        AssetManager assetManager = activity.getAssets();
        WebHandler rootHandler = newStaticAssetHandler(assetManager, ROOT_URI + "/index.html");
        manager.register(ROOT_URI, rootHandler);
        manager.register(ROOT_URI + "/", rootHandler);
//        addAssetWebHandlers(manager, assetManager, "terminal");

//        webServerAttached = true;
    }
}
