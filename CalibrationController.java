/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package hip.positioning.system;

import java.net.URL;
import java.util.ResourceBundle;
import javafx.event.*;
import javafx.fxml.*;
import javafx.scene.*;
import javafx.scene.control.*;
import javafx.scene.image.*;
import javafx.scene.layout.*;
import javafx.scene.text.TextAlignment;
import javafx.stage.Stage;

/**
 * FXML Controller class
 *
 * @author Tristan
 */
public class CalibrationController implements Initializable {
    int nextStep = 0;
    @FXML
    Label stepLabel;
    
    @FXML
    Label directions;
    
    @FXML
    ImageView imageView;
    
    @FXML 
    Button nextStepBtn;
    
//    @FXML
//    Image image = new Image("..\\..\\..\\newpackage\\step1.png");
    
    @FXML 
    private void goToNextStep(ActionEvent event) {
        if (nextStep == 0) {
            stepLabel.setText("Step 2");
            directions.setText("Fully open and close brace and press button to continue.");
            directions.setWrapText(true);
            directions.setTextAlignment(TextAlignment.JUSTIFY);
        }
        else if (nextStep == 1) {
            stepLabel.setText("Step 3");
            directions.setText("Lay brace on flat surface until green light advances.");
            directions.setWrapText(true);
            directions.setTextAlignment(TextAlignment.JUSTIFY);
        }
        else if (nextStep == 2) {
            stepLabel.setText("Step 4");
            directions.setWrapText(true);
            directions.setText("Complete one full rotation of knee brace about "
                + "long axis, pause every 45 degrees for three seconds. Hold still " +
                  "or lay flat once complete until green light advances");
            
            directions.setTextAlignment(TextAlignment.JUSTIFY);
        }
        else if (nextStep == 3) {
            stepLabel.setText("Step 5");
            directions.setText("Place brance on patient, wave leg in figure 8 until "
                + "green light advances.");
            directions.setWrapText(true);
            directions.setTextAlignment(TextAlignment.JUSTIFY);
        }
        else if (nextStep == 4) {
            stepLabel.setText("Step 6");
            directions.setText("Place patient in reference configuration and " +
               "press button to finish calibration.");
        }
        else if (nextStep == 5) {
           stepLabel.setText("Calibration completed!");
           directions.setText("");
        }
        else if (nextStep == 6) {
            Stage stage = (Stage) nextStepBtn.getScene().getWindow();
            stage.close();
        }
        nextStep++;
    }
    
    /**
     * Initializes the controller class.
     */
    @Override
    public void initialize(URL url, ResourceBundle rb) {
        // TODO
        stepLabel.setText("Step 1");
            directions.setText("Press to begin calibration.");
            directions.setWrapText(true);
            directions.setTextAlignment(TextAlignment.JUSTIFY);
            
    }    
    
}