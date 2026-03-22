const SUPABASE_URL = window.ENV.SUPABASE_URL;
const SUPABASE_KEY = window.ENV.SUPABASE_KEY;
const EMAIL = window.ENV.EMAIL;
const PASSWORD = window.ENV.PASSWORD;

const RADIUS = 3.55;
const HEIGHT = 3;

let rawData = [];
let filteredData = [];
let refreshTimerId = null;
let chart = null;

const tableBody = document.getElementById("data-table-body");
const refreshSelect = document.getElementById("refresh-interval-select");
const chartCanvas = document.getElementById('water-chart');
const dateFrom = document.getElementById('dateFrom');
const dateUntil = document.getElementById('dateUntil');
const calculationValues = document.getElementById('calculation-values');

async function getData() {

  const { data, error } = await client
        .from('water_levels')
        .select()
        .order('measure_date', { ascending: false });

    console.log(data)

    rawData = data.map(r => ({
        timestamp: new Date(r.measure_date).getTime(),
        value: Math.round(((HEIGHT - Number(r.distance)) * Math.pow(RADIUS, 2) * Math.PI) * 10) / 10
      }));

    console.log(rawData);

    const from = dateFrom.value ? new Date(dateFrom.value).getTime() : -Infinity;
    const until = dateUntil.value ? new Date(dateUntil.value).getTime() : Infinity;

    filteredData = rawData.filter(r => {
      return r.timestamp >= from && r.timestamp <= until;
    })
      
    renderTable();
    renderChart();
}

function renderTable() {
  tableBody.innerHTML = "";
  for (const item of filteredData) {
    const tr = document.createElement("tr");

    const dateCell = document.createElement("td");
    const date = new Date(item.timestamp);
    dateCell.textContent = date.toLocaleString();

    const valueCell = document.createElement("td");
    valueCell.textContent = item.value;

    tr.appendChild(dateCell);
    tr.appendChild(valueCell);
    tableBody.appendChild(tr);
  }
}

function setAutoRefresh() {
  const interval = parseInt(refreshSelect.value, 10);
  if (refreshTimerId) {
    clearInterval(refreshTimerId);
    refreshTimerId = null;
  }
  if (interval > 0) {
    refreshTimerId = setInterval(getData, interval);
  }
}

function renderChart() {

  const labels = filteredData.map(item =>
    new Date(item.timestamp).toLocaleString()
  );
  const values = filteredData.map(item => item.value);

  if (!chart) {
    chart = new Chart(chartCanvas, {
      type: 'line',
      data: {
        labels: labels,
        datasets: [{
          label: 'Wasserstand in m³',
          data: values,
          borderWidth: 1,
          responsive: true,
          maintainAspectRatio: false,
        }]
      },
      options: {
        scales: {
          x: {
            ticks: {
              color: "white",
              maxRotation: 0,
              autoSkip: true,
              maxTicksLimit: 6,
              font: {
                size: 11
              }
            }
          },
          y: {
            beginAtZero: true,
            grid: { color: "#1f2937" },
            ticks: {
              color: "white",
              font: {
                size: 11
              }
            }
          }
        },
        plugins: {
            tooltip: {
              mode: "index",
              intersect: false
            },
            legend: {
              labels: {
                color: "white",
                font: { size: 11 }
              }
            }
          }
      }
    });
  } else {
    chart.data.labels = labels;
    chart.data.datasets[0].data = values;
    chart.update();
  }
}

function clearDates() {
  dateFrom.value = '';
  dateUntil.value = '';
  getData();
}

refreshSelect.addEventListener("change", setAutoRefresh);

calculationValues.textContent = `Wasserreservoir: Höhe: ${HEIGHT}m | Radius: ${RADIUS}m`;

const { createClient } = supabase;
client = createClient(SUPABASE_URL, SUPABASE_KEY);

client.auth.signOut();

client.auth.signInWithPassword({
    email: EMAIL,
    password: PASSWORD
}).then(({ data, error }) => {
    if (error) {
        console.log('Auto-login failed:', error.message);
    } else {
        console.log('Auto-logged in:', data.user.email);
        getData();
    }
});
