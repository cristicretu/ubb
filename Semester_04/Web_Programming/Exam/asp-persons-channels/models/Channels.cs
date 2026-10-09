using System.ComponentModel.DataAnnotations;
using System.ComponentModel.DataAnnotations.Schema;

namespace ProjectManagement.Models
{
    public class Channels
    {
        [Key]
        public int Id { get; set; }

        [MaxLength(100)]
        public string? Name { get; set; }

        [MaxLength(100)]
        public string? Description { get; set; }

        [MaxLength(500)]
        public string? Subscribers { get; set; }

        public int SubscriberCount 
        { 
            get 
            {
                if (string.IsNullOrEmpty(Subscribers)) return 0;
                return Subscribers.Split(';', StringSplitOptions.RemoveEmptyEntries).Length;
            }
        }

        public int AddSubscriber(string subscriber)
        {
            var oldSubscribers = Subscribers;
            var currentDate = DateTime.Now.ToString("dd.MM.yyyy");
            var newSubscribers = $"{oldSubscribers};{subscriber}|{currentDate}";
            Subscribers = newSubscribers;
            return SubscriberCount;
        }

        public bool UpdateSubscriber(string subscriber)
        {
            if (string.IsNullOrEmpty(Subscribers)) return false;
            
            var subscriberEntries = Subscribers.Split(';', StringSplitOptions.RemoveEmptyEntries).ToList();
            var currentDate = DateTime.Now.ToString("dd.MM.yyyy");
            
            for (int i = 0; i < subscriberEntries.Count; i++)
            {
                var entry = subscriberEntries[i];
                if (entry.Contains('|'))
                {
                    var parts = entry.Split('|');
                    if (parts.Length >= 2 && parts[0] == subscriber)
                    {
                        subscriberEntries[i] = $"{subscriber}|{currentDate}";
                        Subscribers = string.Join(";", subscriberEntries);
                        return true;
                    }
                }
            }
            
            return false; // Subscriber not found
        }

        public int? OwnerId { get; set; }

        public Persons? Owner { get; set; }
    }
} 