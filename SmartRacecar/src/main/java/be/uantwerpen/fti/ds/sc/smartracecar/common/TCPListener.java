package be.uantwerpen.fti.ds.sc.smartracecar.common;

import java.io.IOException;

public interface TCPListener {
    String parseTCP(String message) throws IOException;
}
