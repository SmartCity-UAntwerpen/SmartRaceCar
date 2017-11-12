package be.uantwerpen.fti.ds.sc.smartracecar.common;

import java.io.IOException;

/**
 * TCP Socket functionality interface to be called on the other modules.
 */
public interface TCPListener {

    /**
     * Interfaced method to parse TCP message socket callback is triggered by incoming message.
     *
     * @param message received TCP socket message string
     * @return a return answer to be send back over the socket.
     */
    String parseTCP(String message) throws IOException;
}
