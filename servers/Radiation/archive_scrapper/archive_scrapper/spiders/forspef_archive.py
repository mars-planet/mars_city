import scrapy


class ForspefArchiveScrapper(scrapy.Spider):

    name = "forspef"

    def start_requests(self):
        #url = 'http://tromos.space.noa.gr/forspef/archive/results.php?date1=%272016-01-01%2000:00:00%27&date2=%272017-01-01%2023:59:59%27'
        url = 'http://tromos.space.noa.gr/forspef/archive/results.php?date1=%272015-08-01%2000:00:00%27&date2=%272017-07-01%2023:59:59%27'
        yield scrapy.Request(url=url, callback=self.parse)

    def parse(self, response):
        
        tables = response.css("table")
        SEPprobsD = tables[3].css("tr")[2:]
        
        dates = []
        probs =  []
        
        count = 0
        for j in tables:
            count = count + 1
            if count == 4:
                break
            for i in j.css("tr")[2:]:
                data = i.css('td')
                date = data[0].css("td::text").extract_first()
                prob = data[-1].css("td::text").extract_first()
                if float(prob)>0.25:
                    dates.append(date)
                    probs.append(prob)
        print dates
        print "\n"
        print probs

