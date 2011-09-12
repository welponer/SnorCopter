package AeroQuad.AQ_Configurator;

import AeroQuad.AQ_Configurator.SerialCommunicator.ISerialCommunicator;
import AeroQuad.AQ_Configurator.SerialCommunicator.SerialCommunicator;
import AeroQuad.AQ_Configurator.UI.PCRemoteControllerMainFrame;

import javax.swing.*;

public class Starter
{
    public Starter()
    {
        init();
    }

    private void init()
    {
        final ISerialCommunicator communicator = new SerialCommunicator();
        final PCRemoteControllerMainFrame mainFrame = new PCRemoteControllerMainFrame(communicator);
        mainFrame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
    }


    public static void main(String[] args)
    {
        SwingUtilities.invokeLater(new Runnable()
        {
            public void run()
            {
                new Starter();
            }
        });
    }
}
