using System;
using System.Data;
using System.Data.SqlClient;
using System.Windows.Forms;

namespace DBMS
{
    public partial class Form1 : Form
    {
        SqlConnection cs = new SqlConnection("Data Source=np:\\\\.\\pipe\\LOCALDB#C4384798\\tsql\\query; Initial Catalog = formula1; Integrated Security = True");
        SqlDataAdapter daTeams, daDrivers;
        DataSet dsTeams = new DataSet(), dsDrivers = new DataSet();
        int selectedTeamId = -1;

        public Form1()
        {
            InitializeComponent();
        }

        private void Form1_Load(object sender, EventArgs e)
        {
            LoadTeams();
        }

        private void LoadTeams()
        {
            try
            {
                daTeams = new SqlDataAdapter("SELECT * FROM Teams", cs);
                dsTeams.Clear();
                daTeams.Fill(dsTeams);
                dataGridView1.DataSource = dsTeams.Tables[0];
            }
            catch (Exception ex)
            {
                MessageBox.Show("Error loading teams: " + ex.Message);
            }
        }

        private void dataGridView1_CellClick(object sender, DataGridViewCellEventArgs e)
        {
            if (e.RowIndex >= 0)
            {
                Console.WriteLine("m,essi");
                selectedTeamId = Convert.ToInt32(dataGridView1.Rows[e.RowIndex].Cells["teamId"].Value);
                LoadDrivers(selectedTeamId);
            }
        }

        private void LoadDrivers(int teamId)
        {
            try
            {
                daDrivers = new SqlDataAdapter("SELECT * FROM Drivers WHERE teamId = @teamId", cs);
                daDrivers.SelectCommand.Parameters.AddWithValue("@teamId", teamId);

                dsDrivers.Clear();
                daDrivers.Fill(dsDrivers);
                dataGridView2.DataSource = dsDrivers.Tables[0];
            }
            catch (Exception ex)
            {
                MessageBox.Show("Error loading drivers: " + ex.Message);
            }
        }

        private void button2_Click(object sender, EventArgs e)
        {
           
        }

        private void dataGridView1_CellContentClick(object sender, DataGridViewCellEventArgs e)
        {

        }

        private void button1_Click(object sender, EventArgs e)
        {
            if (selectedTeamId == -1)
            {
                MessageBox.Show("Please select a team first.");
                return;
            }

            try
            {
                SqlCommand insertCmd = new SqlCommand("INSERT INTO Drivers (driverId, driverNumber, firstName, lastName, country, teamId) VALUES (@id, @num, @first, @last, @country, @teamId)", cs);
                insertCmd.Parameters.AddWithValue("@id", int.Parse(textBox1.Text));
                insertCmd.Parameters.AddWithValue("@num", int.Parse(textBox2.Text));
                insertCmd.Parameters.AddWithValue("@first", textBox3.Text);
                insertCmd.Parameters.AddWithValue("@last", textBox4.Text);
                insertCmd.Parameters.AddWithValue("@country", textBox5.Text);
                insertCmd.Parameters.AddWithValue("@teamId", selectedTeamId);

                cs.Open();
                insertCmd.ExecuteNonQuery();
                cs.Close();

                MessageBox.Show("Driver added successfully!");
                LoadDrivers(selectedTeamId);
            }
            catch (Exception ex)
            {
                MessageBox.Show("Error inserting driver: " + ex.Message);
                cs.Close();
            }
        }

        private void button4_Click_1(object sender, EventArgs e)
        {
            try
            {
                SqlCommandBuilder builder = new SqlCommandBuilder(daDrivers);
                daDrivers.Update(dsDrivers.Tables[0]);
                MessageBox.Show("Driver details updated successfully!");
                LoadDrivers(selectedTeamId);
            }
            catch (Exception ex)
            {
                MessageBox.Show("Error updating driver: " + ex.Message);
            }
        }

        private void button3_Click_1(object sender, EventArgs e)
        {
            if (dataGridView2.SelectedRows.Count > 0)
            {
                int driverId = Convert.ToInt32(dataGridView2.SelectedRows[0].Cells["driverId"].Value);
                try
                {
                    SqlCommand deleteCmd = new SqlCommand("DELETE FROM Drivers WHERE driverId = @id", cs);
                    deleteCmd.Parameters.AddWithValue("@id", driverId);

                    cs.Open();
                    deleteCmd.ExecuteNonQuery();
                    cs.Close();

                    MessageBox.Show("Driver removed successfully!");
                    LoadDrivers(selectedTeamId);
                }
                catch (Exception ex)
                {
                    MessageBox.Show("Error removing driver: " + ex.Message);
                    cs.Close();
                }
            }
            else
            {
                MessageBox.Show("Please select a driver to remove.");
            }
        }

        private void button3_Click(object sender, EventArgs e)
        {
           
        }

        private void button4_Click(object sender, EventArgs e) 
        {
           
        }
    }
}
