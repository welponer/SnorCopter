package AeroQuad.AQ_Configurator.UI;

import AeroQuad.AQ_Configurator.SerialCommunicator.ISerialCommunicator;
import AeroQuad.AQ_Configurator.UI.ArtificialHorizon.ArtificialHorizonController;
import AeroQuad.AQ_Configurator.UI.MotorsPanel.MotorsPanel;
import AeroQuad.AQ_Configurator.UI.MotorsPanel.MotorsPanelController;
import AeroQuad.AQ_Configurator.UI.ShipConnectionPanel.ShipConnectionPanel;
import AeroQuad.AQ_Configurator.UI.ShipConnectionPanel.ShipConnectionPanelController;
import AeroQuad.AQ_Configurator.srcimport.ArtificialHorizon.PanelGUI;

import javax.swing.*;
import java.awt.*;

public class PCRemoteControllerMainFrame extends JFrame
{
    public PCRemoteControllerMainFrame(final ISerialCommunicator communicator)
    {
        initUi(communicator);
    }

    private void initUi(final ISerialCommunicator communicator)
    {
        final JPanel mainPanel = new JPanel(new BorderLayout());


        final ShipConnectionPanel connectionPanel = new ShipConnectionPanel(new ShipConnectionPanelController(communicator));
        final JPanel connectionPanelContainer = new JPanel(new GridLayout(2, 1));
        connectionPanelContainer.add(connectionPanel);

        mainPanel.add(connectionPanelContainer);

        final PanelGUI haPanel = new PanelGUI("kalman");
        new ArtificialHorizonController(haPanel, communicator);
        mainPanel.add(haPanel, BorderLayout.WEST);

        final MotorsPanel motorPanel = new MotorsPanel(new MotorsPanelController(communicator));
        mainPanel.add(motorPanel, BorderLayout.SOUTH);
        getContentPane().add(mainPanel);
        pack();
        setVisible(true);
    }
}
