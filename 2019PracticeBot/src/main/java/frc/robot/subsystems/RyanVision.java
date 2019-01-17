public class RyanVision
{
    private int [][] parameters; 
    private int [][] denselayers;
    private int conversiondifference;
    private int pixle_difference;
    private int [][]condensed;
    private int [] condensed_test;

    RyanVision(int [][][][] pictures)
    {
        condensed= new int [pictures.length()][(pictures[0].length()*pictures[0][0].length()*pictures[0][0][0].length())];
        denselayers= new int [(pictures[0].length()*pictures[0][0].length()*pictures[0][0][0].length())][100];
        parameters= new int [100][1];
        for (int i=0; i<denselayers.length(); i++)
            for (int j=0; j<denselayers[i].length(); i++)
            {    
                int random= (int) 100*Math.random();
                denselayers[i][j]=random;
            }
        for (int i=0; i<parameters.length(); i++)
            for (int j=0; j<parameters[i].length(); i++)
            {    
                int random= (int) 100*Math.random();
                parameters[i][j]=random;
            }
        //Condenses the picture into a 1 array form of the picture
        for (int picture=0; picture<pictures.length(); picture++)
        {
            for (int rgb=0; rgb<pictures[picture].length(); rgb++)
            {
                for (int rows=0; rows<pictures[picture][rgb].length(); rows++)
                {
                    for (int colums=0; colums<pictures[picture][rgb][rows].length(); colums++)
                {
                    condensed[picture][(rgb*((rows*colums)+colums))+(rows*colums)+colums]=pictures[picture][rgb][rows][colums];
                }
                }
            }
        }
        //The immage should go from a (N, 3, R, C) --> (N, 3*R*C) 
        //ex. (N, 3, 32, 32) --> (N, 3072)
    }
    RyanVision(int [][][] picture)
    {
        condensed_test= new int [(pictures.length()*pictures[0].length()*pictures[0][0].length())]
        //Condenses the picture into a 1 array form of the picture
        for (int rgb=0; rgb<pictures.length(); rgb++)
        {
            for (int rows=0; rows<picture[rgb].length(); rows++)
            {
                for (int colums=0; colums<picture[rgb][rows].length(); colums++)
            {
                condensed_test[(rgb*((rows*colums)+colums))+(rows*colums)+colums]=picture[rgb][rows][colums];
            }
            }
        }
        //The immage should go from a (N, 3, R, C) --> (N, 3*R*C) 
        //ex. (N, 3, 32, 32) --> (N, 3072)
    }
    public int movedistance()
    {
        return pixle_difference*conversiondifference;
    }
    private int propagate(int [][] picture)
    {
        return parameters*(picture*denselayers);
    }
    
}