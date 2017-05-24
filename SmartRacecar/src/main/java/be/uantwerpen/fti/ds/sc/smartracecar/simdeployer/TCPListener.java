package be.uantwerpen.fti.ds.sc.smartracecar.simdeployer;

import java.io.IOException;

interface TCPListener {
    String parseTCP(String message) throws IOException;
}
