package IsPrime;

@SuppressWarnings("serial")
public class MyValueException extends Exception {

    public MyValueException(String msg){
        super(msg);
    }

    public MyValueException(Exception ex){
        super(ex);
    }
}
