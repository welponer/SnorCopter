package AeroQuad.AQ_Configurator.UI.ArtificialHorizon;


import AeroQuad.AQ_Configurator.SerialCommunicator.ISerialCommunicator;
import AeroQuad.AQ_Configurator.SerialCommunicator.Message.SensorsDataMessage;
import AeroQuad.AQ_Configurator.SerialCommunicator.Message.SerialMessage;
import AeroQuad.AQ_Configurator.srcimport.ArtificialHorizon.ArtificialHorizon;
import AeroQuad.AQ_Configurator.srcimport.ArtificialHorizon.PanelGUI;

import java.beans.PropertyChangeEvent;
import java.beans.PropertyChangeListener;

public class ArtificialHorizonController implements AeroQuad.AQ_Configurator.UI.ArtificialHorizon.IArtificialHorizonController
{
    private final PanelGUI _haPanel;
    public ArtificialHorizonController(final PanelGUI haPanel, final ISerialCommunicator communicator)
    {
        _haPanel = haPanel;
        communicator.addListener(SerialMessage.SENSORS_DATA_MESSAGE_NAME,new PropertyChangeListener()
        {
            @Override
            public void propertyChange(PropertyChangeEvent evt)
            {
                final SensorsDataMessage message = (SensorsDataMessage)evt.getNewValue();
                processMessage(message);
            }
        });
    }

    private void processMessage(final SensorsDataMessage message)
    {
        ArtificialHorizon.rollValueFiltered = (int)message.getRoll()*-1;
        ArtificialHorizon.pitchValueFiltered = (int)message.getPitch();
        _haPanel.repaint();
    }
}
